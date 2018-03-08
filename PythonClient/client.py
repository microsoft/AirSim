import asynchat
import asyncore
import socket
import threading
 
class ChatClient(asynchat.async_chat):
 
    def __init__(self, host, port):
        asynchat.async_chat.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect((host, port))
        print('Connected')
        self.set_terminator(b'\r\nDONEPACKET\r\n')
        self.buffer = []
 
    def collect_incoming_data(self, data):
        self.buffer.append(data.decode('ASCII'))
 
    def found_terminator(self):
        msg = ''.join(self.buffer)
        print('Received: %s' % msg)
        self.buffer = []
 
client = ChatClient('localhost', 5050)
 
comm = threading.Thread(target=asyncore.loop)
comm.daemon = True
comm.start()
 
while True:
    msg = input('> ')
    client.push((msg).encode("ASCII") + b'\r\nDONEPACKET\r\n')