# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from AirSimClient import *
import pprint
import tempfile
from math import *
from scipy.misc import imsave

def get_vec_dist (x_dst, y_dst, x_src, y_src):
    vec = np.array([x_dst,y_dst] - np.array([x_src, y_src]))
    dist = math.sqrt(vec[0]**2 + vec[1]**2)
    return vec/dist, dist

def get_angle (x_dst, y_dst, x_src, y_src):
    angle = math.atan2(y_dst-y_src,x_dst-x_src)
    if angle > math.pi:
        angle -= 2*math.pi
    elif angle < -math.pi:
        angle += 2*math.pi
    return angle

def get_local_goal (v, pos, theta):
    return goal

pp = pprint.PrettyPrinter(indent=4)

client = MultirotorClient()
client.confirmConnection()

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
#AirSimClientBase.wait_key('Press any key to start')

depth_cutoff = 50
roi_w = 100
roi_h = 20
coll_thres = 25
yaw = 0;
step = 0.1
x = 40
y = 3
goal = [120,3] 
timer = 0
time_obs = 50
bObstacle = False

w_mtx = np.ones((roi_h,roi_w))
#for j in range(0,roi_w):
#    for i in range(j,roi_h-j):
#        w_mtx[j:roi_h-j,i:roi_w-i] = (j+1)*(j+1)

#print (w_mtx)

for z in range(10000): # do few times
    #client.simSetPose(Pose(Vector3r(z*step, 5, -1), AirSimClientBase.toQuaternion(0, 0, 0)), True) #pole crash
    x = x + step*cos(yaw)
    y = y + step*sin(yaw)
    client.simSetPose(Pose(Vector3r(x, y, -1), AirSimClientBase.toQuaternion(0, 0, yaw)), True) #pole crash
    #time.sleep(1)

    responses = client.simGetImages([
        ImageRequest(1, AirSimImageType.DepthPlanner, True)])
   
    response = responses[0]

    # get numpy array
    img1d = response.image_data_float

    #print ('Avg: ',np.average(img1d))
    #print ('Min: ',np.min(img1d))
    #print ('Max: ',np.max(img1d))
    #print('Img Sz: ',np.size(img1d))

    # reshape array to 4 channel image array H X W
    img2d = np.reshape(img1d,(response.height, response.width)) 
    img2d_viz = np.reciprocal(img2d)
    #img2d_grad = np.gradient(img2d_viz)
    #img2d[img2d > depth_cutoff] = depth_cutoff

    w = response.width
    h = response.height

    img2d_box = img2d[int((h-roi_h)/2):int((h+roi_h)/2),int((w-roi_w)/2):int((w+roi_w)/2)]
    img2d_box = np.multiply(img2d_box,w_mtx)
    
    #print(img2d_box)
    print ('Box Avg: ',np.average(img2d_box))
    #print ('Box Min: ',np.min(img2d_box))
    #print ('Box Max: ',np.max(img2d_box))
    #print('Box Img Sz: ',np.size(img2d_box))

    target_angle = get_angle(goal[0],goal[1],x,y)
    target_vec,target_dist = get_vec_dist(goal[0],goal[1],x,y)
    if (bObstacle):
        timer = timer + 1
        if timer > time_obs:
            bObstacle = False
            timer = 0
    else:
        yaw = target_angle
    print (target_angle,target_vec,target_dist,x,y,goal[0],goal[1])

    if (np.average(img2d_box) < coll_thres):
        img2d_box_l = img2d_box = img2d[int((h-roi_h)/2):int((h+roi_h)/2),int((w-roi_w)/2)-50:int((w+roi_w)/2)-50]
        img2d_box_r = img2d_box = img2d[int((h-roi_h)/2):int((h+roi_h)/2),int((w-roi_w)/2)+50:int((w+roi_w)/2)+50]
        img2d_box_l_avg = np.average(np.multiply(img2d_box_l,w_mtx))
        img2d_box_r_avg = np.average(np.multiply(img2d_box_r,w_mtx))
        print('left: ', img2d_box_l_avg)
        print('right: ', img2d_box_r_avg)
        if img2d_box_l_avg > img2d_box_r_avg:
            ##Go LEFT
            #y_offset = y_offset-1
            yaw = yaw - radians(10)
            bObstacle = True
        else:
            ##Go RIGHT
            #y_offset = y_offset+1
            yaw = yaw + radians(10)
            bObstacle = true
        print('yaw: ', yaw)

    # write to png 
    imsave(os.path.normpath(os.path.join(tmp_dir, "depth_" + str(z) + '.png')), img2d_viz)
    #imsave(os.path.normpath(os.path.join(tmp_dir, "grad_" + str(z) + '.png')), img2d_grad[1])
    #imsave(os.path.normpath(os.path.join(tmp_dir, "depth_crop_" + str(z) + '.png')), img2d_box)

    #pose = client.simGetPose()
    #pp.pprint(pose)
    if (target_dist < 1):
        print('Target reached.')
        AirSimClientBase.wait_key('Press any key to continue')
        break

    #time.sleep(5)

# currently reset() doesn't work in CV mode. Below is the workaround
client.simSetPose(Pose(Vector3r(0, 0, 0), AirSimClientBase.toQuaternion(0, 0, 0)), True)
