import msgpack
from tornado import tcpserver
from tornado.iostream import IOStream

import msgpackrpc.message
from msgpackrpc.error import RPCError, TransportError


class BaseSocket(object):
    def __init__(self, stream, encodings):
        self._stream = stream
        self._packer = msgpack.Packer(encoding=encodings[0], default=lambda x: x.to_msgpack())
        self._unpacker = msgpack.Unpacker(encoding=encodings[1])

    def close(self):
        self._stream.close()

    def send_message(self, message, callback=None):
        self._stream.write(self._packer.pack(message), callback=callback)

    def on_read(self, data):
        self._unpacker.feed(data)
        for message in self._unpacker:
            self.on_message(message)

    def on_message(self, message, *args):
        msgsize = len(message)
        if msgsize != 4 and msgsize != 3:
            raise RPCError("Invalid MessagePack-RPC protocol: message = {0}".format(message))

        msgtype = message[0]
        if msgtype == msgpackrpc.message.REQUEST:
            self.on_request(message[1], message[2], message[3])
        elif msgtype == msgpackrpc.message.RESPONSE:
            self.on_response(message[1], message[2], message[3])
        elif msgtype == msgpackrpc.message.NOTIFY:
            self.on_notify(message[1], message[2])
        else:
            raise RPCError("Unknown message type: type = {0}".format(msgtype))

    def on_request(self, msgid, method, param):
        raise NotImplementedError("on_request not implemented");

    def on_response(self, msgid, error, result):
        raise NotImplementedError("on_response not implemented");

    def on_notify(self, method, param):
        raise NotImplementedError("on_notify not implemented");


class ClientSocket(BaseSocket):
    def __init__(self, stream, transport, encodings):
        BaseSocket.__init__(self, stream, encodings)
        self._transport = transport
        self._stream.set_close_callback(self.on_close)

    def connect(self):
        self._stream.connect(self._transport._address.unpack(), self.on_connect)

    def on_connect(self):
        self._stream.read_until_close(self.on_read, self.on_read)
        self._transport.on_connect(self)

    def on_connect_failed(self):
        self._transport.on_connect_failed(self)

    def on_close(self):
        self._transport.on_close(self)

    def on_response(self, msgid, error, result):
        self._transport._session.on_response(msgid, error, result)


class ClientTransport(object):
    def __init__(self, session, address, reconnect_limit, encodings=('utf-8', None)):
        self._session = session
        self._address = address
        self._encodings = encodings
        self._reconnect_limit = reconnect_limit;

        self._connecting = 0
        self._pending = []
        self._sockets = []
        self._closed  = False

    def send_message(self, message, callback=None):
        if len(self._sockets) == 0:
            if self._connecting == 0:
                self.connect()
                self._connecting = 1
            self._pending.append((message, callback))
        else:
            sock = self._sockets[0]
            sock.send_message(message, callback)

    def connect(self):
        stream = IOStream(self._address.socket(), io_loop=self._session._loop._ioloop)
        socket = ClientSocket(stream, self, self._encodings)
        socket.connect();

    def close(self):
        for sock in self._sockets:
            sock.close()

        self._connecting = 0
        self._pending = []
        self._sockets = []
        self._closed  = True

    def on_connect(self, sock):
        self._sockets.append(sock)
        for pending, callback in self._pending:
            sock.send_message(pending, callback)
        self._pending = []

    def on_connect_failed(self, sock):
        if self._connecting < self._reconnect_limit:
            self.connect()
            self._connecting += 1
        else:
            self._connecting = 0
            self._pending = []
            self._session.on_connect_failed(TransportError("Retry connection over the limit"))

    def on_close(self, sock):
        # Avoid calling self.on_connect_failed after self.close called.
        if self._closed:
            return

        if sock in self._sockets:
            self._sockets.remove(sock)
        else:
            # Tornado does not have on_connect_failed event.
            self.on_connect_failed(sock)


class ServerSocket(BaseSocket):
    def __init__(self, stream, transport, encodings):
        BaseSocket.__init__(self, stream, encodings)
        self._transport = transport
        self._stream.read_until_close(self.on_read, self.on_read)

    def on_close(self):
        self._transport.on_close(self)

    def on_request(self, msgid, method, param):
        self._transport._server.on_request(self, msgid, method, param)

    def on_notify(self, method, param):
        self._transport._server.on_notify(method, param)


class MessagePackServer(tcpserver.TCPServer):
    def __init__(self, transport, io_loop=None, encodings=None):
        self._transport = transport
        self._encodings = encodings
        tcpserver.TCPServer.__init__(self, io_loop=io_loop)

    def handle_stream(self, stream, address):
        ServerSocket(stream, self._transport, self._encodings)


class ServerTransport(object):
    def __init__(self, address, encodings=('utf-8', None)):
        self._address = address;
        self._encodings = encodings

    def listen(self, server):
        self._server = server;
        self._mp_server = MessagePackServer(self, io_loop=self._server._loop._ioloop, encodings=self._encodings)
        self._mp_server.listen(self._address.port)

    def close(self):
        self._mp_server.stop()
