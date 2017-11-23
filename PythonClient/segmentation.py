# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from AirSimClient import *

client = MultirotorClient()
client.confirmConnection()

AirSimClientBase.wait_key('Press any key to set all object IDs to 0')
found = client.simSetSegmentationObjectID("[\w]*", 0, True);
print("Done: %r" % (found))

#for block environment

AirSimClientBase.wait_key('Press any key to change one ground object ID')
found = client.simSetSegmentationObjectID("Ground", 20);
print("Done: %r" % (found))

#regex are case insensetive
AirSimClientBase.wait_key('Press any key to change all ground object ID')
found = client.simSetSegmentationObjectID("ground[\w]*", 22, True);
print("Done: %r" % (found))

##for neighbourhood environment

#set object ID for sky
found = client.simSetSegmentationObjectID("SkySphere", 42, True);
print("Done: %r" % (found))

#below doesn't work yet. You must set CustomDepthStencilValue in Unreal Editor for now
AirSimClientBase.wait_key('Press any key to set Landscape object ID to 128')
found = client.simSetSegmentationObjectID("[\w]*", 128, True);
print("Done: %r" % (found))

#get segmentation image in various formats
responses = client.simGetImages([
    ImageRequest(0, AirSimImageType.Segmentation),  #depth visualiztion image
    ImageRequest(0, AirSimImageType.Segmentation, True), #depth in perspective projection
    ImageRequest(0, AirSimImageType.Segmentation, False, False)])  #scene vision image in uncompressed RGBA array
print('Retrieved images: %d', len(responses))

#save segmentation images in various formats
for idx, response in enumerate(responses):
    filename = 'c:/temp/py_seg_' + str(idx)

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        #AirSimClientBase.write_pfm(os.path.normpath(filename + '.pfm'), AirSimClientBase.getPfmArray(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        #AirSimClientBase.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array - numpy demo
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
        img_rgba = img1d.reshape(response.height, response.width, 4) #reshape array to 4 channel image array H X W X 4
        img_rgba = np.flipud(img_rgba) #original image is fliped vertically
        #AirSimClientBase.write_png(os.path.normpath(filename + '.numpy.png'), img_rgba) #write to png 

        #find unique colors
        print(np.unique(img_rgba[:,:,0], return_counts=True)) #red
        print(np.unique(img_rgba[:,:,1], return_counts=True)) #green
        print(np.unique(img_rgba[:,:,2], return_counts=True)) #blue  
        print(np.unique(img_rgba[:,:,3], return_counts=True)) #blue








