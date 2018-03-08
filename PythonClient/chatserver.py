import asynchat
import asyncore
import socket
import threading
 
chat_room = {}

a_list = []

class Test:
    def pr(self, data):
        print("Type " + str(type(self)))
        print("From test " + data)

class ChatHandler(asynchat.async_chat):
    def __init__(self, sock, callback):
        asynchat.async_chat.__init__(self, sock=sock, map=chat_room)
 
        self.set_terminator(b'\n')
        self.buffer = []
        self.callback = callback
 
    def collect_incoming_data(self, data):
        self.buffer.append(data.decode('ASCII'))
 
    def found_terminator(self):
        msg = ''.join(self.buffer)
        print('Received: %s'% msg)
        a_list.append(msg)
        self.callback(msg)
        for handler in chat_room.values():
            if hasattr(handler, 'push'):
                handler.push((msg + '\n').encode('ASCII'))
        self.buffer = []
 
class ChatServer(asyncore.dispatcher):
    def __init__(self, host, port, handler):
        asyncore.dispatcher.__init__(self, map=chat_room)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.bind((host, port))
        self.listen(5)
        self.res_handler = handler
 
    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Incoming connection from %s' % repr(addr))
            handler = ChatHandler(sock, self.res_handler)
 
t = Test()
server = ChatServer('localhost', 5050, t.pr)
 
print('Serving on localhost:5050')
comm = threading.Thread(target= lambda: (asyncore.loop(map = chat_room)))
comm.daemon = True
comm.start()

while True:
    msg = input('> ')
    if (msg == "print"):
        print(a_list)
        continue
    if (msg[:4] == "exec"):
        exec(msg[5:])
        continue
    for handler in chat_room.values():
            if hasattr(handler, 'push'):
                handler.push((msg + '\n').encode('ASCII'))