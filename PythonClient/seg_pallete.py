import numpy
import random
import cv2

def generate_color_palette(numPixelsWide, outputFile):
    random.seed(42)

    palette = numpy.zeros((1, 256 * numPixelsWide, 3))
    possibilities = [list(range(256)), list(range(256)), list(range(256))]

        
    for i in range(3):
        for j in range(256):
            choice = random.sample(possibilities[i], 1)[0]
            possibilities[i].remove(choice)
            palette[0, j * numPixelsWide:(j + 1) * numPixelsWide, i] = choice

    cv2.imwrite(outputFile, palette, [cv2.IMWRITE_PNG_COMPRESSION,0])


if __name__ == '__main__':

    numPixelsWide = 4
    outputFile = 'c:/temp/seg_color_pallet.png'
    generate_color_palette(numPixelsWide, outputFile)