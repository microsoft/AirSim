from AirSimClient import *
import copy
import cv2
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
        self._id = id
        intermidiate_map = PModHCameraInfo.get_camera_type_map()
        for k, value in intermidiate_map.items():
            intermidiate_map[k] = [value, 0]
        self._cameraTypeMap = intermidiate_map
        self.requests = []
    
    def update(self):
        # take and return image requests
        self.requests = []
        for k, value in self._cameraTypeMap.items():
            if value[1] > 0:
                pixels_as_float = True if k in ["depth", "depth_planner", "depth_perspective", "disparity"] else False
                self.requests.append([self._id, k, ImageRequest(self._id, value[0], pixels_as_float, False)])
    
    def get_requests(self):
        return self.requests

    def get_id(self):
        return self._id
    
    def add_image_type(self, typename):
        self._cameraTypeMap[typename][1] += 1

    def remove_image_type(self, typename):
        curr_count = self._cameraTypeMap[typename][1]
        if curr_count > 0:
            self._cameraTypeMap[typename][1] = curr_count - 1

    @staticmethod
    def get_camera_type_map():
        return { 
            "depth": AirSimImageType.DepthVis,
            "depth_planner": AirSimImageType.DepthPlanner,
            "depth_perspective": AirSimImageType.DepthPerspective,
            "segmentation": AirSimImageType.Segmentation,
            "scene": AirSimImageType.Scene,
            "disparity": AirSimImageType.DisparityNormalized,
            "normals": AirSimImageType.SurfaceNormals
        }
'''
    Scene = 0
    DepthPlanner = 1
    DepthPerspective = 2
    DepthVis = 3
    DisparityNormalized = 4
    Segmentation = 5
    SurfaceNormals = 6
'''
    
# Persistent ModuleBase
class PModBase:
    def __init__(self, client, persistent_modules):
        self.client = client
        self.persistent_modules = persistent_modules

    def get_name():
        raise NotImplementedError
    
    def start(self):
        raise NotImplementedError

    def update(self):
        raise NotImplementedError
    
    def stop(self):
        raise NotImplementedError

class PModConstants(PModBase):
    def __init__(self, client, persistent_modules):
        super().__init__(client, persistent_modules)
        self.standard_speed = 5 # 5m/s

    def get_name():
        return 'constants'

    def start(self):
        pass

    def update(self):
        pass

    def stop(self):
        pass

# Persistent Module
class PModMyState(PModBase):
    def __init__(self, client, persistent_modules):
        super().__init__(client, persistent_modules)
        self._state = MultirotorState()
    
    def get_name():
        return 'mystate'

    def start(self):
        pass # not required

    def update(self):
        self._state = self.client.getMultirotorState()
    
    def get_state(self):
        return self._state

    def get_position(self):
        return self._state.kinematics_true.position
    
    def get_orientation(self):
        return self._state.kinematics_true.orientation

# Persistent Module 
class PModCamera(PModBase): # PMod => persistent module
    def __init__(self, client, persistent_modules):
        super().__init__(client, persistent_modules)
        self._cameras = [PModHCameraInfo(i) for i in range(5)]
        self._image_dicts = [PModHCameraInfo.get_camera_type_map() for i in range(5)] + [ImagesInfo(), ]
        self._oldimage_dicts = self._image_dicts
        self._num_image_iter = 0

    def get_name():
        return 'camera'

    def start(self):
        pass # Change default camera settings here

    def update(self):
        self._num_image_iter += 1

        # Update old images
        self._oldimage_dicts = copy.deepcopy(self._image_dicts)

        # Update all cameras 
        for c in self._cameras:
            c.update()
        
        # Collect all image requests
        requests = []
        requestsinfo = []
        for c in self._cameras:
            for req in c.requests:
                requests.append(req[2])
                requestsinfo.append((req[0], req[1]))

        # Execute all requests if any
        if (len(requests) == 0):
            return
         
        responses = self.client.simGetImages(requests)
        assert len(responses) == len(requests)

        # Process responces and save in respective dicts
        for n, res in enumerate(responses):
            i, k = requestsinfo[n]
            self._image_dicts[i][k] = self._extract_image(res)

    def _extract_image(self, response):
        img1d = None
        channels = 4
        if (response.image_type in [1, 2, 3, 4]):
            img1d = np.asarray(response.image_data_float, np.float32)
            #print(img1d)
            channels = 1
        else:
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
        #reshape array to 4 channel image array H X W X channels
        img_rgba = img1d.reshape(response.height, response.width, channels) 
        if channels == 4:
            img_bgra = img_rgba[:,:,[2,1,0,3]]
        else:
            img_bgra = img_rgba
        return Image(img_bgra, response.time_stamp, response.camera_position, response.camera_orientation)
    
    def get_camera(self, camera_id):
        return self._cameras[camera_id]

    def get_image(self, camera_id, image_type):
        img = self._image_dicts[camera_id][image_type]
        if type(img) == Image:
            return img
        else:
            raise ValueError("Image does not exist, did you call add_image_type() on camera " + str(camera_id))

    def get_oldimage(self, camera_id, image_type):
        img = self._oldimage_dicts[camera_id][image_type]
        if type(img) == np.ndarray:
            return img
        else:
            return self.get_image(camera_id, image_type)

    
    def get_image_pair(self, camera_id, image_type):
        return (self.get_image(camera_id, image_type), self.get_oldimage(camera_id, image_type))
        
    ## TODO add other methods of camera orientation etc


# TODO Continue 

# 

# Persistent Module Helper class
class PModHCameraHelper:
    def __init__(self, persistent_modules):
        self.mystate_module = persistent_modules['mystate']
        self.camera_module = persistent_modules['camera']
    
    def update(self):
        self.camera_module._image_dicts[-1].timestamp = time.time()
        self.camera_module._image_dicts[-1].state = self.mystate_module.get_state()


# Dependency Camera Module 
class PModWindowsManager(PModBase):
    def __init__(self, client, persistent_modules):
        super().__init__(client, persistent_modules)
        self.windows = {}
    
    def get_name():
        return 'windows_manager'

    def start(self):
        self.camera_module = self.persistent_modules['camera']

    def update(self):
        for k, fun in self.windows.items():
            cv2.imshow(k, fun())

    def add_window_by_camera(self, camera_id, image_type):
        name = "Camera_" + str(camera_id) + "_" + image_type
        self.camera_module.get_camera(camera_id).add_image_type(image_type)
        self.add_window(name, lambda: self.camera_module.get_image(camera_id, image_type).image_data)

    def remove_window_by_camera(self, camera_id, image_type):
        name = "Camera_" + str(camera_id) + "_" + image_type
        self.camera_module.cameras[camera_id].remove_image_type(image_type)
        self.remove_window(name)

    def add_window(self, name, image_function):
        if name in self.windows.keys():
            raise KeyError("Window already exists")
        self.windows[name] = image_function

    def remove_window(self, name):
        self.windows.pop(name, None)
