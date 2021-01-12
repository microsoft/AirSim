import setup_path
import airsim
import numpy as np
import time
import math
import cv2
import open3d as o3d #pip install open3d; conda install "tornado<4.5.3"

from scipy.sparse import lil_matrix
from scipy.optimize import least_squares



def getColors(pixel_coords, image):
    color = []
    for i in range(len(pixel_coords)):
        x, y = pixel_coords[i].ravel()
        x = min(image.shape[1]-1,max(0,x))  
        y = min(image.shape[0]-1,max(0,y))   
        BGR = ((image[int(y),int(x)].astype(float))/255)
        color.append([BGR[2],BGR[1],BGR[0]])
    color = np.reshape(color,(len(color),3))
    return color
    
def triangulate(pts1, P0, pts2, P1, K, d):
    #initialize lists
    obj_pts = []
    mask = []
    total = 0
    count = 0

    #convert to weird array format that undistort points needs (N, 1, 2) DEPENDS ON OPENCV version
    pts1 = np.reshape(pts1,(len(pts1),1,2))
    pts2 = np.reshape(pts2,(len(pts2),1,2))

    #Normalize and undistort, this will convert the pts from pixel coordinates to camera coordinates (i.e. [-1 > 0 < 1])
    pts1_norm = cv2.undistortPoints(pts1, K, d)
    pts2_norm = cv2.undistortPoints(pts2, K, d)
    pts1_norm = np.reshape(pts1_norm,(len(pts1),2))
    pts2_norm = np.reshape(pts2_norm,(len(pts2),2))
    #pull out Rotation and translation from P1 matrix for reprojection test
    R = np.reshape(P1[:,0:3],(3,3))
    t = np.reshape(P1[:,3],(3,1))

    #for all points
    points3d = []
    for i in range(len(pts1_norm)):
        #triangulate
        point = triangulate_int(pts1_norm[i],P0,pts2_norm[i],P1)
	points3d.append(point)
    points3d = np.reshape(points3d,(len(points3d),3))
    #do a reprojection
    reproj_pts, ret = cv2.projectPoints(points3d,R,t,K,d)
    for i in range(len(reproj_pts)):
	    rx, ry = reproj_pts[i].ravel()
	    ox, oy = pts2[i].ravel() 
	    #get distance between projected and original points
	    distance = math.sqrt((rx-ox)**2+(ry-oy)**2)
	    #test if distance is above threshold
	    if (distance < 5):
		#add point to cloud
		obj_pts.append(points3d[i])
		print(points3d[i])
		count = count + 1
		mask.append([1])
		#aggregate distance for average calculation
		total = total + distance
	    else:
		mask.append([0])

    obj_pts = np.reshape(obj_pts,(len(obj_pts),3))
    mask = np.reshape(mask,(len(mask),1))
    error = total/(len(obj_pts)+1)
    print('Triangulated points',count,'out of',len(pts1_norm),'=',int(count/len(pts1_norm)*100),'%')
    print('Average reprojection error',int(error),'pixel(s)')  
    
    return obj_pts, mask, error
    
def triangulate_int(pix_pt1,P0,pix_pt2,P1):
    #code taken from 'Mastering OpenCV with Practical Computer Vision Projects' Chapter 4
    #https://www.cs.ccu.edu.tw/~damon/photo/,OpenCV/,Mastering_OpenCV.pdf
    #build A matrix
    A = np.array([[pix_pt1[0]*P0[2,0]-P0[0,0],pix_pt1[0]*P0[2,1]-P0[0,1],pix_pt1[0]*P0[2,2]-P0[0,2]],
    [pix_pt1[1]*P0[2,0]-P0[1,0],pix_pt1[1]*P0[2,1]-P0[1,1],pix_pt1[1]*P0[2,2]-P0[1,2]],
    [pix_pt2[0]*P1[2,0]-P1[0,0], pix_pt2[0]*P1[2,1]-P1[0,1],pix_pt2[0]*P1[2,2]-P1[0,2]],
    [pix_pt2[1]*P1[2,0]-P1[1,0], pix_pt2[1]*P1[2,1]-P1[1,1],pix_pt2[1]*P1[2,2]-P1[1,2]]])
    #build B vector
    B = np.array([[-1*(pix_pt1[0]*P0[2,3]-P0[0,3])],
    [-1*(pix_pt1[1]*P0[2,3]-P0[1,3])],
    [-1*(pix_pt2[0]*P1[2,3]-P1[0,3])],
    [-1*(pix_pt2[1]*P1[2,3]-P1[1,3])]])
    #solve for X
    ret, X = cv2.solve(A,B,flags=cv2.DECOMP_SVD)
    #print(X)
    return X

def drawTracks(orig, next, img, color):   
    # Draws the optical flow tracks
    output = img.copy()
    mask = np.zeros_like(output)
    for i, (new, old) in enumerate(zip(next, orig)):
        # Returns a contiguous flattened array as (x, y) coordinates for new point
        a, b = new.ravel()
        # Returns a contiguous flattened array as (x, y) coordinates for old point
        c, d = old.ravel()
        # Draws line between new and old position with green color and 2 thickness
        mask = cv2.line(mask, (a, b), (c, d), color, 2)
        # Draws filled circle (thickness of -1) at new position with green color and radius of 3
        output = cv2.circle(output, (a, b), 3, color, -1)    
    # Overlays the optical flow tracks on the original frame
    output = cv2.add(output, mask)
    return output
    
def getTrackLength(orig,new):
    num_samples  = 10
    rand_pts = np.random.choice(len(new),num_samples,replace=False)
    total = 0
    for i in range(num_samples):
        j = rand_pts[i]
        x1, y1 = orig[j].ravel()
        x2, y2 = new[j].ravel() 
        #get distance between new and original points
        distance = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
        total = total + distance
    avg_dist = total/num_samples
    return avg_dist
    
def getObjectPointsEssential(prev_pts,new_pts,P0,K,d):
    #Get essential matrix to derive R, t, and F
    E, mask_e = cv2.findEssentialMat(prev_pts,new_pts, K, method=cv2.RANSAC, prob=0.999, threshold = 1.0)
    print ("Essential matrix: used ", np.sum(mask_e) ," of total ",len(new_pts),"matches")
    prev_pts = prev_pts[mask_e[:,0]==1]
    new_pts = new_pts[mask_e[:,0]==1]
   
    R1, R2, t1 = cv2.decomposeEssentialMat(E)
    t2 = -1*t1
    
    P1 = []
    #These are the four possible Projection Matrices, only one produces the most correct points
    P1.append(np.hstack((R1,t1))*P0)
    P1.append(np.hstack((R1,t2))*P0)
    P1.append(np.hstack((R2,t1))*P0)
    P1.append(np.hstack((R2,t2))*P0)
    best_obj_pts = []
    best_error = 1000
    for i in range(4):
        #Get 3d world points and associated pixel values for the second image
        obj_pts, mask_tri, error = triangulate(prev_pts, P0, new_pts, P1[i], K, d)
        if(len(obj_pts) > len(best_obj_pts)):            
            best_error = error
            best_obj_pts = obj_pts
            best_mask_tri = mask_tri
            best_P1 = P1[i]
        elif(len(obj_pts) == len(best_obj_pts) & (error < best_error)):
            best_error = error
            best_obj_pts = obj_pts
            best_mask_tri = mask_tri
            best_P1 = P1[i]
    
    prev_pts = prev_pts[best_mask_tri[:,0]==1]
    new_pts = new_pts[best_mask_tri[:,0]==1]
    
    return best_obj_pts, prev_pts, new_pts, best_P1
    
def getObjectPointsPnP(obj_pts,new_pts,P0,K,d,rvec,tvec):
    #SolvePNP
    _, rvec, tvec, inliers  = cv2.solvePnPRansac(obj_pts, pixel_pts, K, d, rvec, tvec, useExtrinsicGuess = True)
    print('SolvePnP used',len(inliers),'points of total', len(pixel_pts),'=',int(len(inliers)/len(pixel_pts)*100),'%')
    #convert pnp rotation to (3x3) with rodrigues method
    R, _ = cv2.Rodrigues(rvec)
    t = tvec 

    #Create second projection matrix
    #P1 = np.dot(K,np.hstack((R, t)))
    P1 = np.hstack((R, t))

    #Now we need to get the full set of matches to apply this projection matrix to
    #Get 3d world points and associated pixel values for the second image
    obj_pts, mask_tri = triangulate(pts1, P0, pts2, P1, K, d)
    
    return obj_pts, mask_tri, P1
    
def eliminateDuplicateObjects(prev_objs, new_features):
    #get all objects and pixel coords from the last image
    prev_objs = np.reshape(prev_objs,(-1,6))
    #a new mask to check for last images objects in the new features
    mask_1 = np.full((len(new_features),4),-1)
    #a new mask to remove these duplicates from the pre_objs_array
    mask_2 = np.ones(len(prev_objs[:,0]))
    count = 0
    for i in range(len(new_features)):
        x,y = new_features[i].ravel()
        for j in range(len(prev_objs)):
            #checks distance between new features and existing object pixels
            if(math.isclose(prev_objs[j,4],x,rel_tol=0.02) & math.isclose(prev_objs[j,5],y,rel_tol=0.03)):
                if (mask_2[j] == 1):    
                    count += 1
                    mask_1[i] = prev_objs[j,0:4]
                    mask_2[j] = 0
                break                          
    #eliminate duplicate objects contained in the pre_objs array
    prev_objs = prev_objs[mask_2==1]
    #create the overall mask
    mask_3 = np.vstack((mask_1,prev_objs[:,0:4]))
    #create a total list of pixel coords consisting of new features and previous pixel coordinates,
    #must be the same length as the object index mask
    new_features = new_features.reshape(len(new_features),2)
    orig = np.vstack((new_features,prev_objs[:,4:6])).reshape(-1, 1, 2).astype(np.float32)
    
    return orig, mask_3
    
def baFun(params, n_cameras, n_points, camera_indices, points_2d):
    """Compute residuals.   
    `params` contains camera parameters and 3-D coordinates.
    """
    cam_list = []
    for i in range(n_cameras):
        begin = (i*6)
        end = (i*6+6)
        cam_list.append(params[begin:end].ravel())
        
    points_3d = []
    for i in range(n_points):
        begin = i*3+(n_cameras*6)
        end = i*3+3+(n_cameras*6)
        points_3d.append(params[begin:end])
    
    proj_list = []  
    for i in range(n_points):
        camera = cam_list[camera_indices[i]]
        R = np.reshape(camera[0:3],(1,3))
        t = np.reshape(camera[3:6],(1,3))
        R, _ = cv2.Rodrigues(R)
        point_proj, ret = cv2.projectPoints(points_3d[i],R,t,K,d)
        proj_list.append(point_proj)
    proj_list = np.reshape(proj_list,(-1,2))
    points_2d = np.reshape(points_2d,(-1,2))
    return (proj_list - points_2d).ravel()
    
def bundle_adjustment_sparsity(n_cameras, n_points, camera_indices, point_indices):
    m = camera_indices.size * 2
    n = n_cameras * 6 + n_points * 3
    A = lil_matrix((m, n), dtype=int)

    i = np.arange(camera_indices.size)
    for s in range(6):
        A[2 * i, camera_indices * 6 + s] = 1
        A[2 * i + 1, camera_indices * 6 + s] = 1

    for s in range(3):
        A[2 * i, n_cameras * 6 + point_indices * 3 + s] = 1
        A[2 * i + 1, n_cameras * 6 + point_indices * 3 + s] = 1

    return A
