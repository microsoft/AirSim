import asynchat
import asyncore
import socket
import threading
import time

class EngageObject:
    def __init__(self, id, addr):
        self.addr = addr
        self.data = b''
        self.id = id
        self.status = -1
        self.done = False

class ChatHandler(asynchat.async_chat):
    def __init__(self, sock, addr, callback, chat_room):
        asynchat.async_chat.__init__(self, sock=sock, map=chat_room)
        self.addr = addr
        self.set_terminator(b'\r\nDONEPACKET\r\n')
        self.buffer = []
        self.callback = callback
 
    def collect_incoming_data(self, data):
        self.buffer.append(data.decode('ASCII'))
 
    def found_terminator(self):
        msg = ''.join(self.buffer)
        print('Received: %s'% msg)
        msg = msg.split(" ")

        engage_object = EngageObject(msg[0], self.addr)

        self.callback(msg[1:], engage_object)
        # for handler in chat_room.values():
        #     if hasattr(handler, 'push'):
        #         handler.push((msg + '\n').encode('ASCII'))
        self.buffer = []
 
class ChatServer(asyncore.dispatcher):
    def __init__(self, host, port, handler, chat_room):
        asyncore.dispatcher.__init__(self, map=chat_room)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.bind((host, port))
        self.listen(5)
        self.res_handler = handler
        self.chat_room = chat_room
 
    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Incoming connection from %s' % repr(addr))
            handler = ChatHandler(sock, addr, self.res_handler, self.chat_room)
            handler.push("Hello".encode("ASCII") + b'\r\nDONEPACKET\r\n')

class ModCommandServer:
    def __init__(self):
        self.engage_object_list = []
        self.chat_room = {}
        server = ChatServer('localhost', 5050, self.process, self.chat_room)
        self.comm = threading.Thread(target= lambda: (asyncore.loop(map=self.chat_room)))
        self.comm.daemon = True
        self.comm.start()
        print('Serving on localhost:5050')

    # Only test method
    def later(self, data, engage_object):
        time.sleep(2)
        engage_object.data = (str(engage_object.id) + ' done').encode('ASCII')
        engage_object.status = 0
        engage_object.done = True


    def process(self, data, engage_object):
        print("Processing test " + str(data))
        # replace here with add_command
        self.engage_object_list.append(engage_object)
        threading.Thread(target=lambda: self.later(data, engage_object)).start()

    def update(self):
        #print("dispatching")
        delete_list = []
        for e in self.engage_object_list:
            if e.done == True:
                for handler in self.chat_room.values():
                    if hasattr(handler, 'push'):
                        packetstr = e.id + " " + str(e.status) + " "
                        packet = packetstr.encode('ASCII') + e.data + b'\r\nDONEPACKET\r\n'
                        handler.push(packet)
                delete_list.append(e)
        for e in delete_list:
            self.engage_object_list.remove(e)

m = ModCommandServer()
while(True):
    m.update()
    time.sleep(0.5)
    # while True:
    #     msg = input('> ')
    #     if (msg == "print"):
    #         print(a_list)
    #         continue
    #     if (msg[:4] == "exec"):
    #         exec(msg[5:])
    #         continue
    #     if (msg == "dispatch"):
    #         m.update()

