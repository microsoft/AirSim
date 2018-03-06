from AirSimClient import *
import cv2
import numpy as np
import time 
import copy

# Extra Classes

class Image:
    def __init__(self, image_data, timestamp, camera_position, camera_orientation):
        self.image_data = image_data # will be np.array in form of bgra
        self.timestamp = timestamp
        self.camera_position = camera_position
        self.camera_orientation = camera_orientation

class ImagesInfo:
    def __init__(self, timestamp = None, state = None):
        self.timestamp = timestamp
        self.state = state

class PModHCameraInfo: #PModH => persistent module helper
    def __init__(self, id):
        self.id = id
        intermidiate_map = PModHCameraInfo.get_camera_type_map()
        for k, value in intermidiate_map.items():
            intermidiate_map[k] = [value, 0]
        self.cameraTypeMap = intermidiate_map
        self.requests = []

    def add_image_type(self, typename):
        self.cameraTypeMap[typename][1] += 1

    def remove_image_type(self, typename):
        curr_count = self.cameraTypeMap[typename][1]
        if curr_count > 0:
            self.cameraTypeMap[typename][1] = curr_count - 1
    
    def update(self):
        # take and return image requests
        self.requests = []
        for k, value in self.cameraTypeMap.items():
            if value[1] > 0:
                self.requests.append([self.id, k, ImageRequest(self.id, value[0], False, False)])

    @staticmethod
    def get_camera_type_map():
        return { 
            "depth": AirSimImageType.DepthVis,
            "segmentation": AirSimImageType.Segmentation,
            "scene": AirSimImageType.Scene,
            "disparity": AirSimImageType.DisparityNormalized #,
            #"normals": AirSimImageType.SurfaceNormals
        }
    
# Persistent ModuleBase
class PModBase:
    def __init__(self, client, persistent_modules):
        self.client = client
        self.persistent_modules = persistent_modules
    
    def start(self):
        raise NotImplementedError

    def update(self):
        raise NotImplementedError
    
    def stop(self):
        raise NotImplementedError

# Persistent Module
class PModMyState(PModBase):
    def __init__(self, client, persistent_modules):
        super().__init__(client, persistent_modules)
        self.state = MultirotorState()
    
    def start(self):
        pass # not required

    def update(self):
        self.state = self.client.getMultirotorState()

# Persistent Module 
class PModCamera(PModBase): # PMod => persistent module
    def __init__(self, client, persistent_modules):
        super().__init__(client, persistent_modules)
        self.cameras = [PModHCameraInfo(i) for i in range(5)]
        self.images_dicts = [PModHCameraInfo.get_camera_type_map() for i in range(5)] + [ImagesInfo(), ]
        self.oldimages_dicts = self.images_dicts
        self.num_image_iter = 0
    
    def _get_image(self, response):
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
        img_rgba = img1d.reshape(response.height, response.width, 4) #reshape array to 4 channel image array H X W X 4
        img_bgra = img_rgba[:,:,[2,1,0,3]]
        return Image(img_bgra, response.time_stamp, response.camera_position, response.camera_orientation)

    def start(self):
        pass # Change default camera settings here

    def update(self):
        self.num_image_iter += 1

        # Update old images
        self.oldimages_dicts = copy.deepcopy(self.images_dicts)

        # Update all cameras 
        for c in self.cameras:
            c.update()
        
        # Collect all image requests
        requests = []
        requestsinfo = []
        for c in self.cameras:
            for req in c.requests:
                requests.append(req[2])
                requestsinfo.append((req[0], req[1]))

        # Execute all requests
        responses = self.client.simGetImages(requests)
        assert len(responses) == len(requests)

        # Process responces and save in respective dicts
        for n, res in enumerate(responses):
            i, k = requestsinfo[n]
            self.images_dicts[i][k] = self._get_image(res)

    ## TODO add other methods of camera orientation etc

# Persistent Module Helper class
class PModHCameraHelper:
    def __init__(self, persistent_modules):
        self.mystate_module = persistent_modules['mystate']
        self.camera_module = persistent_modules['camera']
    
    def update(self):
        self.camera_module.images_dicts[-1].timestamp = time.time()
        self.camera_module.images_dicts[-1].state = self.mystate_module.state


# Dependency Camera Module 
class PModWindowsManager(PModBase):
    def __init__(self, client, persistent_modules):
        super().__init__(client, persistent_modules)
        self.windows = {}
    
    def start(self):
        self.camera_module = self.persistent_modules['camera']

    def update(self):
        for k, fun in self.windows.items():
            cv2.imshow(k, fun())

    def add_window_by_camera(self, camera_id, image_type):
        name = "Camera_" + str(camera_id) + "_" + image_type
        self.camera_module.cameras[0].add_image_type(image_type)
        self.add_window(name, lambda: self.camera_module.images_dicts[camera_id][image_type].image_data)

    def remove_window_by_camera(self, camera_id, image_type):
        name = "Camera_" + str(camera_id) + "_" + image_type
        self.camera_module.cameras[camera_id].remove_image_type(image_type)
        self.remove_window(name)

    def add_window(self, name, image_function):
        self.windows[name] = image_function

    def remove_window(self, name):
        self.windows.pop(name, None)

# BaseCmd 
class BaseCmd:
    def __init__(self, client, persistent_modules, modules, line, engage_object):
        self.client = client
        self.persistent_modules = persistent_modules
        self.modules = modules
        self.line = line
        self.engage_object = engage_object

    def start(self):
        raise NotImplementedError
    
    def update(self):
        raise NotImplementedError
    
    # Inheritable Static Method
    def can_process(line):
        raise NotImplementedError

# Cmd
class CmdMove(BaseCmd):
    def __init__(self, client, persistent_modules, modules, line, engage_object):
        super().__init__(client, persistent_modules, modules, line, engage_object)
        self.final_location = None
        
        # Set default if not specified
        if self.line[1] == 'null':
            self.line[1] == '1'

    def start(self):
        self.mystate_module = self.persistent_modules['mystate']
        locationVec = list(self.mystate_module.state.kinematics_true.position)
        # Process command
        if self.line[0] == 'up':
            locationVec[2] -= float(self.line[1])
        elif self.line[0] == 'down':
            locationVec[2] += float(self.line[1])
        elif self.line[0] == 'forward':
            yaw = AirSimClientBase.toEulerianAngle(self.mystate_module.state.kinematics_true.orientation)[2]
            locationVec[0] += float(self.line[1]) * math.cos(yaw)
            locationVec[1] += float(self.line[1]) * math.sin(yaw)
        elif self.line[0] == 'backword':
            yaw = AirSimClientBase.toEulerianAngle(self.mystate_module.state.kinematics_true.orientation)[2]
            locationVec[0] -= float(self.line[1]) * math.cos(yaw)
            locationVec[1] -= float(self.line[1]) * math.sin(yaw)
        elif self.line[0] == 'right':
            yaw = AirSimClientBase.toEulerianAngle(self.mystate_module.state.kinematics_true.orientation)[2]
            locationVec[0] += float(self.line[1]) * math.sin(yaw)
            locationVec[1] += float(self.line[1]) * math.cos(yaw)
        elif self.line[0] == 'left':
            yaw = AirSimClientBase.toEulerianAngle(self.mystate_module.state.kinematics_true.orientation)[2]
            locationVec[0] -= float(self.line[1]) * math.sin(yaw)
            locationVec[1] -= float(self.line[1]) * math.cos(yaw)
        self.final_location = locationVec

        # Note that this call is cancellable if other movement related call is called
        self.client.moveToPosition(self.final_location[0], self.final_location[1], self.final_location[2], 5, 0)
    
    def update(self):
        locationVec = list(self.mystate_module.state.kinematics_true.position)
        # Check if movement is complete or < 0.5 meters distance, anyway thats offset
        if ((self.final_location[0] - locationVec[0])**2 + (self.final_location[1] - locationVec[1])**2
            + (self.final_location[2] - locationVec[2])**2)**(1/2) < 0.5:
            return True
        return False
        
    def can_process(line):
        if line[0] in ['forward', 'backword', 'up', 'down', 'left', 'right']:
            return True
        return False

# Cmd 
class CmdTakePic(BaseCmd):
    def __init__(self, client, persistent_modules, modules, line, engage_object):
        super().__init__(client, persistent_modules, modules, line, engage_object)
        
    def start(self):
        # Engage pic camera front for now
        self.camera_module = self.persistent_modules['camera']
        self.camera_module.cameras[0].add_image_type('scene')

    def update(self):
        img = self.camera_module.images_dicts[0]['scene'] # Returns Image Object
        # do something with image
        # TODO
        

        # Disengage Camera module
        self.camera_module.camera[0].remove_image_type('scene')

        # update engage_object
        self.engage_object.data = img
        self.engage_object.status = True
        self.engage_object.done = True
        return True

    def can_process(line):
        if line[0] in ['picture']:
            return True
        return False

# Module
class ModBase:
    def __init__(self, client, persistent_modules):
        self.client = client
        self.persistent_modules = persistent_modules
    
    def update(self):
        raise NotImplementedError


# Main Controller
class Controller:
    def __init__(self):
        # Connect Simulator
        self.client = MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        # Persistent Modules
        self.persistent_modules = {}
        self.mystate = PModMyState(self.client, self.persistent_modules)
        self.persistent_modules["mystate"] = self.mystate
        self.camera = PModCamera(self.client, self.persistent_modules)
        self.persistent_modules["camera"] = self.camera
        self.persistent_modules['windows_manager'] = PModWindowsManager(self.client, self.persistent_modules)


        # start all persistent modules
        for k, mod in self.persistent_modules.items():
            mod.start()
        # Persistent Module Helpers
        self.persistent_module_helpers = {}
        self.persistent_module_helpers['camera_helper'] = PModHCameraHelper(self.persistent_modules)

        # Modules 
        self.modules = {}
        

        # Test
        self.persistent_modules['windows_manager'].add_window_by_camera(0, 'scene')
        # End Test

        # Commands
        self.commands = []
        self.command_buffer = []
    

    def control(self):
        while(True):
            # Update persistent modules
            for k in self.persistent_modules.keys():
                self.persistent_modules[k].update()

            # Update persistent module helpers
            for k, mod in self.persistent_module_helpers.items():
                    mod.update()

            # Update current commands
            cpoplist = []
            for c in self.commands:
                ans = c.update()
                if ans == True:
                    cpoplist.append(c)
            for c in cpoplist:
                self.commands.remove(c)

            # Add new commands if any
            if len(self.commands) == 0:
                cmd = 0
                try:
                    cmd = self.command_buffer.pop(0)
                except IndexError:
                    pass
                if cmd != 0:
                    cmd.start()
                    self.commands.append(cmd)

            # Add for cv2.imshow() to work
            key = cv2.waitKey(1) & 0xFF
            if (key == 27 or key == ord('q') or key == ord('x')):
                break

ctrl = Controller()
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
