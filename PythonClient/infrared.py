from AirSimClient import *
import numpy
import cv2
import time

def get_image(x, y, z, pitch, roll, yaw, client):

    

    client.simSetPose(Pose(Vector3r(x, y, z), AirSimClientBase.toQuaternion(pitch, roll, yaw)), True)

    print('id',client.simGetSegmentationObjectID("Roof_Middle_Half_01"))

    success = client.simSetSegmentationObjectID("[\w]*", 0, True);
    #print('success', success)
    success = client.simSetSegmentationObjectID("birch[\w]*", 2, True);
    #print('success', success)
    success = client.simSetSegmentationObjectID("fir[\w]*", 2, True);
    #print('success', success)
    success = client.simSetSegmentationObjectID("hedge[\w]*", 5, True);
    #print('success', success)
    success = client.simSetSegmentationObjectID("tree[\w]*", 2, True);
    #print('success', success)

    #responses = client.simGetImages([ImageRequest(0, AirSimImageType.Segmentation, compress=True)])

    #AirSimClientBase.write_file(os.path.normpath(str(x) + '.png'), responses[0].image_data_uint8)
    #im = cv2.imread(os.path.normpath(str(x) + '.png'))

    responses = client.simGetImages([ImageRequest(0, AirSimImageType.Segmentation, compress=False)])
    img1d = numpy.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
    #print('img1d.shape', img1d.shape)
    im = img1d.reshape(responses[0].height, responses[0].width, 4) #reshape array to 4 channel image array H X W X 4
    #print('img shape', im.shape)


    return Vector3r(x, y, z), AirSimClientBase.toQuaternion(pitch, roll, yaw), im[:,:,:3]


def radiance(absoluteTemperature, emissivity, dx=0.01, response=None):

    #if multiple temperatures and emissivities, please shape them as (numberT/Es, 1), so ndim > 1

    wavelength = numpy.arange(8,14,dx)
    c1 = 1.19104e8 # (2 * 6.62607*10^-34 [Js] * 
                   # (2.99792458 * 10^14 [micron/s])^2 * 10^12 to convert 
                   # denominator from microns^3 to microns * m^2)
    c2 = 1.43879e4 # (hc/k) [micron * K]
    if response is not None:
        radiance = response * emissivity * (c1 / ((wavelength**5) * (numpy.exp(c2 / \
                   (wavelength * absoluteTemperature )) - 1)))
    else:
        radiance = emissivity * (c1 / ((wavelength**5) * (numpy.exp(c2 / \
                   (wavelength * absoluteTemperature )) - 1)))
    if absoluteTemperature.ndim > 1:
        return radiance, numpy.trapz(radiance, dx=dx, axis=1)
    else:
        return radiance, numpy.trapz(radiance, dx=dx)


def get_radiance_lut(tempEmissivity, pallet, response):

    numObjects = tempEmissivity.shape[0]
    objectNameNumLUT = tempEmissivity[:,0].flatten() #object ID corresponds to column, object name is what's in that location
    #print(pallet[:numObjects])

    #print(tempEmissivity)

    tempEmissivity = tempEmissivity[:,1:].astype(numpy.float64)

    #print(tempEmissivity[:,0], tempEmissivity[:,1])

    #L = numpy.zeros(256)
    #L[:numObjects] = radiance(tempEmissivity[:,1].reshape((-1,1)), tempEmissivity[:,2].reshape((-1,1))).flatten() #object ID corresponds to column, radiance is what's in that location
    L = radiance(tempEmissivity[:,0].reshape((-1,1)), tempEmissivity[:,1].reshape((-1,1)), response=response)[1].flatten() #object ID corresponds to column, radiance is what's in that location
    L = ((L / L.max()) * 255).astype(numpy.uint8)

    #print('L', L)

    radianceLUT = numpy.zeros((256,256,256), dtype=numpy.uint8)
    for obj in range(numObjects):
        radianceLUT[pallet[obj][0], pallet[obj][1], pallet[obj][2]] = L[obj]

    #print(radianceLUT[numpy.where(radianceLUT != 0)])
    #print(numpy.where(radianceLUT != 0))

    return radianceLUT, objectNameNumLUT


def main(numImages, startX, startY, startZ, pitch, roll, yaw, radianceLUT, client):
    i = 0
    cv2.namedWindow('im')
    while i < numImages:
        print('get image')
        vector, angle, im = get_image(startX, startY, startZ, pitch, roll, yaw, client)
        print('have image')
#        print(im.shape)
        #b,g,r = cv2.split(im)
        #im = cv2.merge((r,g,b))
        print(numpy.unique(im[:,:,0], return_counts=True)) #red
        print(numpy.unique(im[:,:,1], return_counts=True)) #green
        print(numpy.unique(im[:,:,2], return_counts=True)) #blue
#        print(im.shape)
#        print(im[:,:,0].dtype)
        
#        im = np.flipud(img_rgba) #original image is fliped vertically
#        print('im', im.shape)
#        print(radianceLUT.dtype, radianceLUT.shape, radianceLUT[radianceLUT != 0])
        irIm = radianceLUT[im[:,:,0], im[:,:,1], im[:,:,2]]
        print('irIm > 0',irIm[irIm != 0])
        #display = numpy.where(im == 20,255,0)
        print('display image')
        cv2.imshow('im', im)
        cv2.waitKey(0)

        startX = vector.x_val + startX * 2
        startY = vector.y_val + startY * 2
        startZ = vector.z_val
        
        i += 1


if __name__ == '__main__':
    client = MultirotorClient()
    client.confirmConnection()
    time.sleep(10)

    numImages = 10
    startX = -1
    startY = -1
    startZ = -122
    pitch = numpy.radians(270)
    roll = 0
    yaw = 0
    response = numpy.load('camera_response.npy')
    tempEmissivity = numpy.array([['elephant',309,0.96], 
                                  ['human',310,0.985], 
                                  ['tree',273,0.952], 
                                  ['grass',273,0.958], 
                                  ['soil',273,0.914], 
                                  ['shrub',273,0.986]])
    pallet = cv2.imread('seg_color_pallet.png')
    pallet = pallet.reshape((256,3))
    print(list(pallet))
    #compute once, output is LUT for object ID (0 to 255 corresponding to Unreal Engine object IDs) to radiance
    radianceLUT, objectNameNumLUT = get_radiance_lut(tempEmissivity, pallet, response)
    main(numImages, startX, startY, startZ, pitch, roll, yaw, radianceLUT, client)