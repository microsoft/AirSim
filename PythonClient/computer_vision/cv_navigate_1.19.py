# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from AirSimClient import *
import pprint
import tempfile
from math import *
from scipy.misc import imsave

from abc import ABC, abstractmethod
 
#define abstract class to return next vector in the format (x,y,yaw)
class AbstractClassGetNextVec(ABC):
    
    @abstractmethod
    def get_next_vec(self, depth, obj_sz, goal, pos):
        print("Some implementation!")
        return pos,yaw

class ReactiveController(AbstractClassGetNextVec):
    def get_next_vec(self, depth, obj_sz, goal, pos):
        print("Some implementation!")
        return

class AvoidLeft(AbstractClassGetNextVec):

    def __init__(self, hfov=radians(90), coll_thres=5, yaw=0, limit_yaw=5, step=0.1):
        self.hfov = hfov
        self.coll_thres = coll_thres
        self.yaw = yaw
        self.limit_yaw = limit_yaw
        self.step = step

    def get_next_vec(self, depth, obj_sz, goal, pos):

        [h,w] = np.shape(depth)
        [roi_h,roi_w] = compute_bb((h,w), obj_sz, self.hfov, self.coll_thres)

        # compute vector, distance and angle to goal
        t_vec, t_dist, t_angle = get_vec_dist_angle (np.array([goal.position.x_val, goal.position.y_val, goal.position.z_val]), np.array([pos.position.x_val, pos.position.y_val, pos.position.z_val]))

        # compute box of interest
        img2d_box = img2d[int((h-roi_h)/2):int((h+roi_h)/2),int((w-roi_w)/2):int((w+roi_w)/2)]

        # scale by weight matrix (optional)
        #img2d_box = np.multiply(img2d_box,w_mtx)
    
        # detect collision
        if (np.min(img2d_box) < coll_thres):
            self.yaw = self.yaw - radians(self.limit_yaw)
        else:
            self.yaw = self.yaw + min (t_angle-self.yaw, radians(self.limit_yaw))

        pos.position.x_val = pos.position.x_val + self.step*cos(self.yaw)
        pos.position.y_val = pos.position.y_val + self.step*sin(self.yaw)

        return pos.position, self.yaw,t_dist


#compute resultant normalized vector, distance and angle
def get_vec_dist_angle (goal, pos):
    vec = goal - pos
    dist = math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
    cross = np.cross(goal,pos)
    dot = np.dot(goal,pos)

    angle = math.atan2(math.sqrt(cross[0]**2 + cross[1]**2 + cross[2]**2), dot)

    angle = math.atan2(vec[1],vec[0])
    #print (angle)

    if angle > math.pi:
        angle -= 2*math.pi
    elif angle < -math.pi:
        angle += 2*math.pi
    return vec/dist, dist, angle

def get_local_goal (v, pos, theta):
    return goal

#compute bounding box size
def compute_bb(image_sz, obj_sz, hfov, distance):
    vfov = hfov2vfov(hfov,image_sz)
    box_h = ceil(obj_sz[0] * image_sz[0] / (math.tan(hfov/2)*distance*2))
    box_w = ceil(obj_sz[1] * image_sz[1] / (math.tan(vfov/2)*distance*2))
    return box_h, box_w

#convert horizonal fov to vertical fov
def hfov2vfov(hfov, image_sz):
    aspect = image_sz[0]/image_sz[1]
    vfov = 2*math.atan( tan(hfov/2) * aspect)
    return vfov

#matrix with all ones
def equal_weight_mtx(roi_h,roi_w):
    return np.ones((roi_h,roi_w))

#matrix with max weight in center and decreasing linearly with distance from center
def linear_weight_mtx(roi_h,roi_w):
    w_mtx = np.ones((roi_h,roi_w))
    for j in range(0,roi_w):
        for i in range(j,roi_h-j):
            w_mtx[j:roi_h-j,i:roi_w-i] = (j+1)
    return w_mtx

#matrix with max weight in center and decreasing quadratically with distance from center
def square_weight_mtx(roi_h,roi_w):
    w_mtx = np.ones((roi_h,roi_w))
    for j in range(0,roi_w):
        for i in range(j,roi_h-j):
            w_mtx[j:roi_h-j,i:roi_w-i] = (j+1)*(j+1)
    return w_mtx

def print_stats(img):
    print ('Avg: ',np.average(img))
    print ('Min: ',np.min(img))
    print ('Max: ',np.max(img))
    print('Img Sz: ',np.size(img))

def generate_depth_viz(img,thres=0):
    if thres > 0:
        img[img > thres] = thres
    else:
        img = np.reciprocal(img)
    return img 


pp = pprint.PrettyPrinter(indent=4)

client = MultirotorClient()
client.confirmConnection()

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
#print ("Saving images to %s" % tmp_dir)
#AirSimClientBase.wait_key('Press any key to start')

#Define start position, goal and size of UAV
startPose = Pose(Vector3r(0,5,-1), AirSimClientBase.toQuaternion(0, 0, 0)); #start pose x,y,z, pitch,roll,yaw
goalPose = Pose(Vector3r(120,0,-1), AirSimClientBase.toQuaternion(0, 0, 0)); #final pose x,y,z, pitch,roll,yaw

uav_size = [0.29*3,0.98*2] #height:0.29 x width:0.98 - allow some tolerance

#Define parameters and thresholds
hfov = radians(90)
coll_thres = 5
yaw = 0
limit_yaw = 5
step = 0.1

#initial position
client.simSetPose(startPose, True)
currentPose = startPose

#predictControl = AvoidLeftIgonreGoal(hfov, coll_thres, yaw, limit_yaw, step)
predictControl = AvoidLeft(hfov, coll_thres, yaw, limit_yaw, step)

for z in range(10000): # do few times
    
    #time.sleep(1)

    # get response
    responses = client.simGetImages([
        ImageRequest(1, AirSimImageType.DepthPlanner, True)])
    response = responses[0]

    # get numpy array
    img1d = response.image_data_float

    # reshape array to 2D array H X W
    img2d = np.reshape(img1d,(response.height, response.width)) 
    
    [currentPose.position,yaw,target_dist] = predictControl.get_next_vec(img2d, uav_size, goalPose, currentPose)
    currentPose.orientation = AirSimClientBase.toQuaternion(0, 0, yaw)
    client.simSetPose(currentPose, True)

    if (target_dist < 1):
        print('Target reached.')
        AirSimClientBase.wait_key('Press any key to continue')
        break

    # write to png 
    #imsave(os.path.normpath(os.path.join(tmp_dir, "depth_" + str(z) + '.png')), generate_depth_viz(img2d,5))

    #pose = client.simGetPose()
    #pp.pprint(pose)
    #time.sleep(5)

# currently reset() doesn't work in CV mode. Below is the workaround
client.simSetPose(Pose(Vector3r(0, 0, 0), AirSimClientBase.toQuaternion(0, 0, 0)), True)






#################### OLD CODE
#    timer = 0
#    time_obs = 50
#    bObstacle = False

#    if (bObstacle):
#        timer = timer + 1
#        if timer > time_obs:
#            bObstacle = False
#            timer = 0
#    else:
#        yaw = target_angle

#    print (target_angle,target_vec,target_dist,x,y,goal[0],goal[1])


#    if (np.average(img2d_box) < coll_thres):
#        img2d_box_l = img2d_box = img2d[int((h-roi_h)/2):int((h+roi_h)/2),int((w-roi_w)/2)-50:int((w+roi_w)/2)-50]
#        img2d_box_r = img2d_box = img2d[int((h-roi_h)/2):int((h+roi_h)/2),int((w-roi_w)/2)+50:int((w+roi_w)/2)+50]
#        img2d_box_l_avg = np.average(np.multiply(img2d_box_l,w_mtx))
#        img2d_box_r_avg = np.average(np.multiply(img2d_box_r,w_mtx))
#        print('left: ', img2d_box_l_avg)
#        print('right: ', img2d_box_r_avg)
#        if img2d_box_l_avg > img2d_box_r_avg:
#            ##Go LEFT
#            #y_offset = y_offset-1
#            yaw = yaw - radians(10)
#            bObstacle = True
#        else:
#            ##Go RIGHT
#            #y_offset = y_offset+1
#            yaw = yaw + radians(10)
#            bObstacle = true
#        print('yaw: ', yaw)