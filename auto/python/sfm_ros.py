import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
import math
import struct

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import point_cloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

from sfm import getColors
from sfm import triangulate
from sfm import triangulate_int
from sfm import drawTracks
from sfm import getTrackLength
from sfm import getObjectPointsEssential
from sfm import eliminateDuplicateObjects
from sfm import baFun
from sfm import bundle_adjustment_sparsity

from scipy.sparse import lil_matrix
from scipy.optimize import least_squares

from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_multiply, quaternion_from_matrix

class mapping():

	def __init__(self):

   		rospy.init_node('mapping', anonymous=True)

		self.bridge = CvBridge()
		self.tfBuffer = tf2_ros.Buffer()
			listener = tf2_ros.TransformListener(self.tfBuffer)

		self.image = []
		self.pose = PoseStamped()
		self.K = []
		self.d = []
		self.cam_width =  []
		self.cam_height = []
		self.rotation = []
		self.translation = []
		self.tracking = False

		self.img_curr = []
		self.img_prev = []

		self.features_orig = []
		self.features_prev = []

		self.obj_mask = []

		self.reset = True

		#INITIALIZE FEATURE MATCHING PARAMETERS#
		self.maxCorners = 500 #Maximum number of corners to return. If there are more corners than are found, the strongest of them is returned
		self.qualityLevel = 0.01 #For example, if best corner has measure = 1500, and qualityLevel=0.01 , then corners with quality<15 are rejected.
		self.minDistance = 10 #Minimum possible Euclidean distance between the returned corners.
		self.lk_params = dict(winSize = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

		#rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
		rospy.Subscriber('orb_slam2_mono/pose', PoseStamped, self.poseCallback)
		rospy.Subscriber('/airsim/base_link/camera/image_raw', Image, self.imageCallback)
		rospy.Subscriber('/airsim/base_link/camera', CameraInfo, self.camInfoCallback)
		self.cloud_pub = rospy.Publisher("cloud", PointCloud2, queue_size=10)
		self.test_pose = rospy.Publisher("test_pose", PoseStamped, queue_size=10)

		print('waiting on topics...')
		rospy.wait_for_message('/airsim/base_link/camera/image_raw', Image)

		#self.cam_width = self.img_curr.shape[0]
		#self.cam_height = self.img_curr.shape[1]
		print('K: ', self.K)
		print('D: ', self.D)
		rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
		print('connected')
		print(float(self.pose.header.stamp.secs)+float(self.pose.header.stamp.nsecs)/1000000000.0,float(self.header.stamp.secs)+float(self.header.stamp.nsecs)/1000000000.0)
		self.run()

	def poseCallback(self, data):
		self.pose = data
		#self.test_pose.publish(self.pose)
		q = (self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w)
		r = quaternion_matrix(q)
		#print(r)
		self.rotation = r[0:3,0:3]
		self.translation = np.array(([self.pose.pose.position.x],[self.pose.pose.position.y],[self.pose.pose.position.z]))
		#print(self.translation)
		
	def imageCallback(self, data):
		self.header = data.header
		self.img_curr = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

	def run(self):
		#TAKE NEW IMAGE#
		rawImage2 = self.img_curr
		if (rawImage2 == None):
			print("Camera is not returning image, please check airsim for error messages")
			sys.exit(0)
		else:
			img2 = self.img_curr
			gray2 = cv2.cvtColor(img2, cv2.COLOR_RGB2GRAY)

		#GET NEW FEATURE LOCATIONS#
		next, status, error = cv2.calcOpticalFlowPyrLK(gray, gray2, prev, None, **lk_params)
		error = error[status[:,0] == 1]
		#First, filter out the points with high error 
		new = next[status[:,0] == 1]
		new = new[error[:,0] < 10]
		#Update the original list of features to reflect pixels that have been lost in the flow
		orig = orig[status[:,0] == 1]
		orig = orig[error[:,0] < 10]
		#update the object mask
		obj_mask = obj_mask[status[:,0] == 1]
		obj_mask = obj_mask[error[:,0] < 10]
		
		# Updates previous good feature points
		prev = new.reshape(-1, 1, 2) 
		
		#Optional visualization
		output = drawTracks(orig.astype(int), new, img2, (0, 255, 0))
		cv2.imshow('Tracks',output)
		gray = gray2
		
		#IF SIGNIFICANT CHANGE FROM PREVIOUS IMAGE# 
		avg_track_len = getTrackLength(orig,new)
		if (avg_track_len > (img.shape[0]/5) or (len(orig)/orig_count < 0.8)):
			print('update, image #:',img_idx)
			#Time to calculate SFM
			
			prev_pts = orig.reshape(len(orig),2)
			new_pts = new.reshape(len(new),2)
			mask_inf = np.zeros(len(new_pts))
			'''Lets check for infiniti points'''
			for i in range(len(new_pts)):
				x1, y1 = prev_pts[i].ravel()
				x2, y2 = new_pts[i].ravel() 
				#get distance between new and original points
				distance = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
				if (distance >= 5):
					mask_inf[i] = 1
			
			new_pts = new_pts[mask_inf==1] 
			obj_mask = obj_mask[mask_inf==1] 
			print(len(prev_pts)-len(new_pts),'"infinite" eliminated')
			prev_pts = prev_pts[mask_inf==1]	   

			#Is this the first calculation?
			if (img_idx == 0):
				#initial P0
				P0 = np.hstack((np.eye(3, 3), np.zeros((3, 1))))
				#get first set of objects from essential matrix
				obj_pts, prev_pts, new_pts, P = getObjectPointsEssential(prev_pts,new_pts,P0,K,d)

				#Check quality of pts
				if (len(obj_pts) > 15):
					cam_objs = []
					for i in range(len(obj_pts)):
						cam_objs.append(np.hstack((i,obj_pts[i].ravel(),prev_pts[i].ravel())))
					all_objs.append(cam_objs)								
					cam_objs = []
					for i in range(len(obj_pts)):
						cam_objs.append(np.hstack((i,obj_pts[i].ravel(),new_pts[i].ravel())))
						obj_idx += 1
					all_objs.append(cam_objs)
					img_idx += 1			   
					all_P_mats.append(P0)
					all_P_mats.append(P)
					P0 = P
					#get a new set of features to track
					new_features = cv2.goodFeaturesToTrack(gray, maxCorners, qualityLevel, minDistance)
					orig, obj_mask = eliminateDuplicateObjects(all_objs[img_idx],new_features)
					t = np.reshape(P0[:,3],(1,3))
					t = t * np.array([1,-1,1])
					#Get colors from detected pixels for coloring pointcloud
					colors = getColors(new_pts,img2.copy())
					all_pts = obj_pts
					all_colors = colors
					all_pts = np.vstack((all_pts,np.reshape(t,(1,3))))
					all_colors = np.vstack((all_colors,np.array([0.,1.,0.])))
				else:	
					orig = cv2.goodFeaturesToTrack(gray, maxCorners, qualityLevel, minDistance)
				
				
					
			else:#get camera matrices from PNP
				#SolvePNP
				masked_objs = obj_mask[obj_mask[:,0]!=-1]
				pixel_pts = new_pts[obj_mask[:,0]!=-1]
				obj_pts = masked_objs[:,1:4]
				
				_, rvec, tvec, inliers  = cv2.solvePnPRansac(obj_pts, pixel_pts, K, d, flags=cv2.SOLVEPNP_EPNP)
				print('SolvePnP used',len(inliers),'points of total', len(pixel_pts),'=',int(len(inliers)/len(pixel_pts)*100),'%')
				#convert pnp rotation to (3x3) with rodrigues method
				P1 = np.hstack((cv2.Rodrigues(rvec)[0], tvec))
	   
				#Now we need to get the full set of matches to apply this projection matrix to
				#Get 3d world points and associated pixel values for the second image
				new_objs, mask_tri, error = triangulate(prev_pts, P0, new_pts, P1, K, d)
				
				obj_mask = obj_mask[mask_tri[:,0]==1]
				prev_pts = prev_pts[mask_tri[:,0]==1]
				new_pts = new_pts[mask_tri[:,0]==1]
			   
				cam_objs = []
				for i in range(len(new_objs)):
					if (obj_mask[i,0] != -1):
						cam_objs.append(np.hstack((obj_mask[i,0],new_objs[i].ravel(),new_pts[i].ravel())))  
					else:
						cam_objs.append(np.hstack((obj_idx,new_objs[i].ravel(),new_pts[i].ravel())))	
						obj_idx += 1			   
				all_objs.append(cam_objs)
				img_idx += 1
				all_P_mats.append(P1)
				P0 = P1
				#get a new set of features to track
				new_features = cv2.goodFeaturesToTrack(gray, maxCorners, qualityLevel, minDistance)
				orig, obj_mask = eliminateDuplicateObjects(all_objs[img_idx],new_features)
				t = np.reshape(P0[:,3],(1,3))
				t = t * np.array([1,-1,1])
				#Get colors from detected pixels for coloring pointcloud
				colors = getColors(new_pts,img2.copy())
				all_pts = np.vstack((all_pts,new_objs))
				all_colors = np.vstack((all_colors,colors))
				all_pts = np.vstack((all_pts,np.reshape(t,(1,3))))
				all_colors = np.vstack((all_colors,np.array([1.,0.,0.])))
			
			print('colro\points',len(all_pts),len(all_colors))
			pcd = o3d.geometry.PointCloud()
			pcd.points = o3d.utility.Vector3dVector(all_pts)
			pcd.colors = o3d.utility.Vector3dVector(all_colors.astype(float))
			o3d.io.write_point_cloud("pointcloud.ply", pcd)	
			
			frames = 3
			if (False):
			#if ((img_idx > frames) & ((img_idx-1) % frames == 0)):
				#Lets do bundle adjustment
				
				n_cameras = frames

				ba_objs = []			
				camera_params = []			
				for i in range(frames):
					ba_objs.extend(np.reshape(all_objs[(img_idx-frames-1+i)],(-1,6)))
					P = all_P_mats[(img_idx-frames-1+i)]
					R, _ = cv2.Rodrigues(P[:,0:3])
					t = np.reshape(P[:,3],(1,3))
					camera_params.extend(np.hstack((R.ravel(),t.ravel())))
				ba_objs = np.reshape(ba_objs,(-1,6))
				camera_params = np.reshape(camera_params,(-1,6))
				print(camera_params)

				camera_indices = np.empty_like(ba_objs[:,0]).astype(int)
				next = 0
				prev = 0
				for i in range(frames):
					next += len(all_objs[(img_idx-frames-1+i)])
					camera_indices[prev:next] = i
					prev = next
					
				points_3d = np.reshape(ba_objs[:,1:4],(-1,3))
				points_2d = np.reshape(ba_objs[:,4:6],(-1,2))
				point_indices = np.reshape(ba_objs[:,0],(-1,1))
	  
				n_points = len(points_3d)

				n = 6 * n_cameras + 3 * n_points
				m = 2 * n_points

				print("n_cameras: {}".format(n_cameras))
				print("n_points: {}".format(n_points))
				print("Total number of parameters: {}".format(n))
				print("Total number of residuals: {}".format(m))
				
				#ba_thread = myThread(1,"thread1",n_cameras,n_points, camera_indices, point_indices)
				
				x0 = np.hstack((camera_params.ravel(), points_3d.ravel()))
				f0 = baFun(x0, n_cameras, n_points, camera_indices, points_2d)
				
				plt.plot(f0)
				#plt.show()

				A = bundle_adjustment_sparsity(n_cameras, n_points, camera_indices, point_indices)
				res = least_squares(baFun, x0, jac_sparsity=A, verbose=2, x_scale='jac', ftol=1e-4, method='trf',
						args=(n_cameras, n_points, camera_indices, points_2d))
				
				plt.plot(res.fun)
				plt.show()
			prev = orig
			orig_count = len(orig)
		  
		 
		key = cv2.waitKey(1) & 0xFF;
		if (key == 27 or key == ord('q') or key == ord('x')):
			break;  

if __name__ == '__main__':
	mapping()
