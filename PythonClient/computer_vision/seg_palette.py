import numpy
import random

# requires Python 3.5.3 :: Anaconda 4.4.0
# pip install opencv-python
import cv2
import pprint


def generate_color_palette(numPixelsWide, outputFile):
    random.seed(42)

    palette = numpy.zeros((1, 256 * numPixelsWide, 3))
    possibilities = [list(range(256)), list(range(256)), list(range(256))]

    colors = [[0] * 3 for i in range(256)]

    choice = 0
    j = 0
    for i in range(3):
        palette[0, j * numPixelsWide : (j + 1) * numPixelsWide, i] = choice
        colors[j][i] = choice

    for i in range(3):
        for j in range(1, 255):
            choice = random.sample(possibilities[i], 1)[0]
            possibilities[i].remove(choice)
            palette[0, j * numPixelsWide : (j + 1) * numPixelsWide, i] = choice
            colors[j][i] = choice

    choice = 255
    j = 255
    for i in range(3):
        palette[0, j * numPixelsWide : (j + 1) * numPixelsWide, i] = choice
        colors[j][i] = choice

    cv2.imwrite(outputFile, palette, [cv2.IMWRITE_PNG_COMPRESSION, 0])

    rgb_file = open("rgbs.txt", "w")
    for j in range(256):
        rgb_file.write("%d\t%s\n" % (j, str(list(reversed(colors[j])))))
    rgb_file.close()


if __name__ == "__main__":
    numPixelsWide = 4
    outputFile = "seg_color_palette.png"
    generate_color_palette(numPixelsWide, outputFile)
