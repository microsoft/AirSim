from AirSimClient import *
# CmdBase 
class CmdBase:
    def __init__(self, client, persistent_modules, modules, line, engage_object):
        self.client = client
        self.persistent_modules = persistent_modules
        self.modules = modules
        self.line = line
        self.engage_object = engage_object
        self.command = line[0]

    def start(self):
        raise NotImplementedError
    
    def update(self):
        raise NotImplementedError

    def get_persistent_module(self, name):
        return self.persistent_modules[name]
    
    # Inheritable Static Method
    def can_process(line):
        raise NotImplementedError

# Cmd
class CmdMove(CmdBase):
    def __init__(self, client, persistent_modules, modules, line, engage_object = None):
        super().__init__(client, persistent_modules, modules, line, engage_object)
        self.final_location = None
        self.constants_module = self.get_persistent_module('constants')
        # Set default if not specified
        self.distance_param = line[1]
        if self.distance_param == 'null':
            self.distance_param == '1m'
        self.param_mode = self.distance_param[-1] 
        self.distance_param = self.distance_param[:-1]
        if self.param_mode == 's':
            self.distance_param = str(float(self.distance_param) * self.constants_module.standard_speed)
        #print(self.command + " " + self.distance_param)

    def start(self):
        self.mystate_module = self.get_persistent_module('mystate')
        locationVec = list(self.mystate_module.get_position())
        offset = [0, 0, 0]
        print(locationVec)
        # Process command
        yaw = AirSimClientBase.toEulerianAngle(self.mystate_module.get_orientation())[2]
        if self.command == 'up':
            offset[2] -= float(self.distance_param)
        elif self.command == 'down':
            offset[2] += float(self.distance_param)
        elif self.command == 'forward':
            offset[0] += float(self.distance_param) * math.cos(yaw)
            offset[1] += float(self.distance_param) * math.sin(yaw)
        elif self.command == 'backward':
            print("Going back")
            offset[0] -= float(self.distance_param) * math.cos(yaw)
            offset[1] -= float(self.distance_param) * math.sin(yaw)
            print(offset)
        elif self.command == 'right':
            offset[0] += float(self.distance_param) * math.sin(yaw)
            offset[1] += float(self.distance_param) * math.cos(yaw)
        elif self.command == 'left':
            offset[0] -= float(self.distance_param) * math.sin(yaw)
            offset[1] -= float(self.distance_param) * math.cos(yaw)
        

        # add to location
        locationVec[0] += offset[0]
        locationVec[1] += offset[1]
        locationVec[2] += offset[2]
        self.final_location = locationVec
        print(self.final_location)

        # Note that this call is cancellable if other movement related call is called
        self.client.moveToPosition(self.final_location[0], self.final_location[1], self.final_location[2],
            self.constants_module.standard_speed, 0)
    
    def update(self):
        locationVec = list(self.mystate_module.get_position())
        # Check if movement is complete or < 0.5 meters distance, anyway thats offset
        if ((self.final_location[0] - locationVec[0])**2 + (self.final_location[1] - locationVec[1])**2
            + (self.final_location[2] - locationVec[2])**2)**(1/2) < 0.5:
            print("inside " + str(self.engage_object))
            if self.engage_object != None:
                print("done with " + str(self.engage_object.id))
                self.engage_object.status = 0
                self.engage_object.done = True
            return True
        return False
        
    def can_process(line):
        if line[0] in ['forward', 'backward', 'up', 'down', 'left', 'right']:
            return True
        return False

# Cmd 
class CmdTakePic(CmdBase):
    def __init__(self, client, persistent_modules, modules, line, engage_object):
        super().__init__(client, persistent_modules, modules, line, engage_object)
        
    def start(self):
        # Engage pic camera front for now
        self.camera_module = self.get_persistent_module('camera')
        self.camera_module.get_camera(0).add_image_type('scene')

    def update(self):
        img = self.camera_module.get_image(0, 'scene') # Returns Image Object
        # do something with image
        # TODO

        # Disengage Camera module
        self.camera_module.get_camera(0).remove_image_type('scene')

        # update engage_object
        self.engage_object.data = img
        self.engage_object.status = True
        self.engage_object.done = True
        return True

    def can_process(line):
        if line[0] in ['camera']:
            return True
        return False

#Cmd 
class CmdReset(CmdBase):
    def __init__(self, client, persistent_modules, modules, line, engage_object):
        super().__init__(client, persistent_modules, modules, line, engage_object)
    
    def start(self):
        self.mystate_module = self.get_persistent_module('mystate')
        self.constants_module = self.get_persistent_module('constants')
        self.client.moveToPosition(0, 0, 0, self.constants_module.standard_speed, 0)
        self.final_location = [0, 0, 0.68]
    
    def update(self):
        locationVec = list(self.mystate_module.get_position())
        if ((self.final_location[0] - locationVec[0])**2 + (self.final_location[1] - locationVec[1])**2
            + (self.final_location[2] - locationVec[2])**2)**(1/2) < 2:
            return True
        return False
    
    def stop(self):
        pass
    
    def can_process(line):
        if line[0] in ['reset']:
            return True
        return False

#Cmd 
class CmdTakeoff(CmdBase):
    def __init__(self, client, persistent_modules, modules, line, engage_object):
        super().__init__(client, persistent_modules, modules, line, engage_object)
    
    def start(self):
        self.client.takeoff(8)
    
    def update(self):
        return True
    
    def stop(self):
        pass
    
    def can_process(line):
        if line[0] in ['takeoff']:
            return True
        return False