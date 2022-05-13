import socket

from tornado.platform.auto import set_close_exec


class Address(object):
    """\
    The class to represent the RPC address.
    """

    def __init__(self, host, port, family=socket.AF_UNSPEC):
        self._host = host
        self._port = port
        self._family = family

    @property
    def host(self):
        return self._host

    @property
    def port(self):
        return self._port

    def unpack(self):
        return (self._host, self._port)

    def socket(self):
        res = socket.getaddrinfo(self._host, self._port, self._family,
                                 socket.SOCK_STREAM, 0, socket.AI_PASSIVE)[0]
        af, socktype, proto, canonname, sockaddr = res
        sock = socket.socket(af, socktype, proto)
        set_close_exec(sock.fileno())
        sock.setblocking(0)
        if af == socket.AF_INET6:
            if hasattr(socket, "IPPROTO_IPV6"):
                sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)

        return sock
