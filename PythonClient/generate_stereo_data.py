from PythonClient import *
import cv2
import numpy as np
import matplotlib.pyplot as plt
from time import sleep
import random
import argparse
import os
import scipy.misc
import StringIO

import read_write_pfm

""" Depth is returned in meters, but the coordinate system is left handed global and uses centimeters """

def convert_plane_depth_to_disp(plane_depth, f=320.0, baseline_meters=1.0):
    disp = f * baseline_meters * (1.0 / plane_depth)
    return disp

def depth_conversion(point_depth, f=320):
    H = point_depth.shape[0]
    W = point_depth.shape[1]
    i_c = np.float(H) / 2 - 1
    j_c = np.float(W) / 2 - 1
    columns, rows = np.meshgrid(np.linspace(0, W-1, num=W), np.linspace(0, H-1, num=H))
    DistanceFromCenter = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
    PlaneDepth = point_depth / (1 + (DistanceFromCenter / f) ** 2) ** (0.5)
    return PlaneDepth

def crop_and_resize(arr=None, desired_size=(540, 960)):
    # Check that arr is (480, 640)
    assert arr.shape[0] == 480
    assert arr.shape[1] == 640

    if len(arr.shape) == 3:
        # Crop to same ratio as (540, 960) which is (360, 640)
        arr = arr[:360, :, :]
        # Resize
        arr_resized = scipy.misc.imresize(arr, (desired_size[0], desired_size[1], 3), interp='nearest')
        return arr_resized
    elif len(arr.shape) == 2:
        # Crop to same ratio as (540, 960) which is (360, 640)
        arr = arr[:360, :]
        # Resize
        arr_resized = scipy.misc.imresize(arr, (desired_size[0], desired_size[1]), interp='nearest')
        return arr_resized
    else:
        return None


if __name__ == '__main__':

    # Parse all arguments
    parser = argparse.ArgumentParser(description='Collect stereo data')
    parser.add_argument('--storage-dir', type=str, help='Directory to write logs and checkpoints', dest='storage_dir', required=True)
    args = parser.parse_args()

    fid = open(os.path.join(args.storage_dir, 'files_list.txt'), 'w')

    # Make storage dir if it does not exist already
    if not os.path.exists(args.storage_dir):
        os.makedirs(args.storage_dir)

    # Connect to the game
    client.connect()
    if not client.isconnected():  # Check if the connection is successfully established
        print 'UnrealCV server is not running. Run the game first!'
    else:

        num_samples = 10

        x_low = -1742.0
        x_high = 570.0
        y_low = -14526.0
        y_high = 17537.2
        z_low = 120.0
        z_high = 150.0

        sleep_interval = 0.1

        for i in range(num_samples):
            print 'Processing image %d'%i
            # Generate a random location
            x = random.uniform(x_low, x_high)
            y = random.uniform(y_low, y_high)
            z = random.uniform(z_low, z_high)

            # Put camera there
            command = 'vset /camera/0/location %f %f %f' % (x, y, z)
            res = client.request(command)
            sleep(sleep_interval)

            # Set pitch, yaw, roll all to be zero
            command = 'vset /camera/0/rotation %f %f %f' % (0.0, 0.0, 0.0)
            res = client.request(command)
            sleep(sleep_interval)

            # Get left camera image
            real_left_filename = '%s/left_%06d.png' % (args.storage_dir, i)
            left_filename = client.request('vget /camera/0/lit %s'%real_left_filename)
            sleep(sleep_interval)
            left = cv2.imread(real_left_filename, cv2.IMREAD_ANYCOLOR)
            left = crop_and_resize(left, desired_size=(540, 960))
            cv2.imwrite(real_left_filename, left)

            # Get depth image (aligned with the left camera)
            depth_str = client.request('vget /camera/0/depth npy')
            if depth_str is None:
                print 'Depth request did not succeed'
            sleep(sleep_interval)
            depth_str_io = StringIO.StringIO(depth_str)
            depth_img = np.load(depth_str_io)

            # Get right camera image
            baseline_cm = 25.0
            command = 'vset /camera/0/location %f %f %f' %(x, y + baseline_cm, z)
            res = client.request(command)
            sleep(sleep_interval)

            real_right_filename = '%s/right_%06d.png' % (args.storage_dir, i)
            right_filename = client.request('vget /camera/0/lit %s'%real_right_filename)
            sleep(sleep_interval)
            right = cv2.imread(real_right_filename, cv2.IMREAD_ANYCOLOR)
            right = crop_and_resize(right, desired_size=(540, 960))
            cv2.imwrite(real_right_filename, right)

            # Convert from point depth to plane depth
            depth = depth_conversion(depth_img, f=320)

            # Parameters for camera
            cx = float(depth.shape[1]) / 2.0 - 1.0
            cy = float(depth.shape[0]) / 2.0 - 1.0
            f = cx

            # Convert the plane depth to disparity
            disparity = convert_plane_depth_to_disp(plane_depth=depth, f=f, baseline_meters=baseline_cm/100.0)
            disparity_filename = '%s/disparity_%06d.pfm'%(args.storage_dir, i)
            disparity = crop_and_resize(arr=disparity, desired_size=(540,960))
            read_write_pfm.save_pfm(disparity_filename, disparity.astype('float32'))

            # Write the tuple to file
            temp_line = real_left_filename + ',' + real_right_filename + ',' + disparity_filename + '\n'
            fid.write(temp_line)

            # plt.subplot(3, 1, 1)
            # plt.imshow(disparity)
            # plt.title('Disparity')
            #
            # plt.subplot(3, 1, 2)
            # plt.imshow(left)
            # plt.title('Left')
            #
            # plt.subplot(3, 1, 3)
            # plt.imshow(right)
            # plt.title('Right')
            #
            # plt.show()

    fid.close()