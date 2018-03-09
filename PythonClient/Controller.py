from AirSimClient import *
from PersistentModules import *
from Commands import *
import cv2
import numpy as np
import time 
import copy

import asynchat
import asyncore
import socket
import threading
# Extra Classes

# Module
class ModBase:
    def __init__(self, client, persistent_modules):
        self.client = client
        self.persistent_modules = persistent_modules
        self.enabled = False
    
    def get_name():
        raise NotImplementedError

    def start(self):
        self.enabled = True
    
    def stop(self):
        self.enabled = False

    def update(self):
        raise NotImplementedError

class EngageObject:
    def __init__(self, id, addr = None):
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
        #print("engage terminate " + str(engage_object))
        self.callback(msg[1:], engage_object)
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

class ModCommandServer(ModBase):
    def __init__(self, client, persistent_modules, add_command):
        super().__init__(client, persistent_modules)
        self.add_command = add_command

        self.engage_object_list = []
        self.chat_room = {}
        self.server = None
        
    def get_name():
        return 'command_server'

    def start(self):
        super().start()
        if (not self.server):
            self.server = ChatServer('localhost', 5050, self.process, self.chat_room)
            self.comm = threading.Thread(target= lambda: (asyncore.loop(map=self.chat_room)))
            self.comm.daemon = True
            self.comm.start()
        print('Serving command API on localhost:5050')

    def stop(self):
        super().stop()
        # Note that this only stops update and not disable server

    # Only test method
    def later(self, msg, engage_object):
        time.sleep(2)
        engage_object.data = (str(engage_object.id) + ' done').encode('ASCII')
        engage_object.status = 0
        engage_object.done = True

    def process(self, msg, engage_object):
        #print("process " + str(engage_object))
        # if server is disabled return msg with fail
        if (not self.enabled):
            engage_object.data = b"Failed: ModCommandServer disabled"
            engage_object.status = -1
            engage_object.done = True
            return
        
        # else
        print("Processing command " + str(msg))
        self.engage_object_list.append(engage_object)
        # replace here with add_command
        #threading.Thread(target=lambda: self.later(msg, engage_object)).start()
        self.add_command(msg, engage_object)


    def update(self):
        delete_list = []
        for e in self.engage_object_list:
            if e.done == True:
                #print(str(e.id) + " done" )
                for handler in self.chat_room.values():
                    if hasattr(handler, 'push'):
                        packetstr = e.id + " " + str(e.status) + " "
                        packet = packetstr.encode('ASCII') + e.data + b'\r\nDONEPACKET\r\n'
                        handler.push(packet)
                delete_list.append(e)
        for e in delete_list:
            self.engage_object_list.remove(e)



# Main Controller
class Controller:
    def __init__(self, persistent_module_classes, persistent_module_helper_classes, module_classes):
        # Connect Simulator
        self.client = MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        # Persistent Modules
        self.persistent_modules = {}
        
        for c in persistent_module_classes:
            self.persistent_modules[c.get_name()] = c(self.client, self.persistent_modules)

        # start all persistent modules
        for k, mod in self.persistent_modules.items():
            mod.start()

        # Persistent Module Helpers
        self.persistent_module_helpers = {}
        self.persistent_module_helpers['camera_helper'] = PModHCameraHelper(self.persistent_modules)

        # Modules 
        self.modules = {}
        self.modules[ModCommandServer.get_name()] = ModCommandServer(self.client, self.persistent_modules, 
                                                                    self.add_command)
        for c in module_classes:
            self.modules[c.get_name()] = c(self.client, self.persistent_modules)

        # Commands
        self.commands = []
        self.commands_buffer = []

        # Vars
        self._iteration = 0

        # Command Classes
        self.command_classes = [
            CmdMove, CmdReset, CmdTakeoff, CmdTakePic
        ]

        # Test
        self.modules['command_server'].start()
        #self.persistent_modules['windows_manager'].add_window_by_camera(0, 'scene')
        #self.persistent_modules['windows_manager'].add_window_by_camera(0, 'depth')
        #self.persistent_modules['windows_manager'].add_window_by_camera(0, 'depth_perspective')

        #self.commands_buffer.append(CmdReset(self.client, self.persistent_modules, self.modules, ['reset', ''], None))
        #self.commands_buffer.append(CmdTakeoff(self.client, self.persistent_modules, self.modules, ['takeoff', ''], None))
        self.commands_buffer.append(CmdMove(self.client, self.persistent_modules, self.modules, ['up', '15m'], None))
        #self.commands_buffer.append(CmdMove(self.client, self.persistent_modules, self.modules, ['left', '69m'], None))
        #self.commands_buffer.append(CmdMove(self.client, self.persistent_modules, self.modules, ['down', '3m'], None))
        # End Test


    def _get_command_object(self, line, engage_object):
        cmd = None
        for c in self.command_classes:
            if c.can_process(line):
                cmd = c(self.client, self.persistent_modules, self.modules, line, engage_object)
        return cmd

    # TODO update this, its a bad practice to assume that it will work -,-, be optimistic though ;)
    def add_command(self, line, engage_object = None):
        cmd = self._get_command_object(line, engage_object)
        if cmd is None:
            engage_object.data = b"Unknown Command"
            engage_object.status = -1
            engage_object.done = True
            return 
        elif type(cmd) in [CmdMove,]:
            print("Detected a move command " + str(line))
            self.commands_buffer.append(cmd)
        else:
            self.commands.append(cmd)
        return True

    def control(self):
        print(list(self.persistent_modules['mystate'].get_position()))
        t_old = time.time()
        while(True):
            self._iteration += 1

            if self._iteration % 100 == 0:
                d_time = time.time() - t_old
                print(str(self._iteration) + " " + str(100/d_time) + " " + 
                    str(list(self.persistent_modules['mystate'].get_position())))
                t_old = time.time()
            # Update persistent modules
            for k in self.persistent_modules.keys():
                self.persistent_modules[k].update()

            # Update persistent module helpers
            for k, mod in self.persistent_module_helpers.items():
                    mod.update()
            
            # Update Modues
            for k, mod in self.modules.items():
                if mod.enabled:
                    mod.update()

            # Update current commands
            cpoplist = []
            for c in self.commands:
                ans = c.update()
                if ans == True:
                    print(list(self.persistent_modules['mystate'].get_position()))
                    cpoplist.append(c)
            for c in cpoplist:
                self.commands.remove(c)

            # Add new commands if any
            if len(self.commands) == 0:
                cmd = 0
                try:
                    cmd = self.commands_buffer.pop(0)
                except IndexError:
                    pass
                if cmd != 0:
                    print("cmd" + cmd.command)
                    cmd.start()
                    self.commands.append(cmd)

            # Add for cv2.imshow() to work
            key = cv2.waitKey(1) & 0xFF
            if (key == 27 or key == ord('q') or key == ord('x')):
                break

persistent_module_classes = [PModConstants, PModMyState, PModCamera, PModWindowsManager]
persistent_module_helper_classes = [PModHCameraHelper,]
module_classes = []

ctrl = Controller(persistent_module_classes, persistent_module_helper_classes, module_classes)
ctrl.control()

'''
Controller
HTTPServer

Mystate

CameraFeed
Stabilize
DQN

Debug
ModWindowsManager
Logging
'''
