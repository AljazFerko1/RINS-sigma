#!/usr/bin/python3

import numpy
import re
from matplotlib import pyplot
import cv2

def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.
    Format specification: http://netpbm.sourceforge.net/doc/pgm.html
    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))


if __name__ == "__main__":
    image = read_pgm("src/exercise7/scripts/map_ok.pgm", byteorder='<')
    # print(image[250])
    partial_image = image[199:298, 205:327]
    # print(partial_image[0:17, 0:17])
    # print(partial_image[-17:-1, 0:17])
    print(partial_image[-17:-1, 15:32])
    
    image_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    image_rgb = image_rgb[199:298, 205:327]
    image_rgb_height = 99
    image_rgb_length = 122
    
    # 39 col je 1.96 v rvizu
    row = 69
    col = 39
    image_rgb[row, :, :] = 0
    image_rgb[:, col, :] = 0
    image_rgb[row, :, 2] = 255
    image_rgb[:, col, 2] = 255
    cv2.imwrite('src/exercise7/scripts/kvadrati/map.png', image_rgb)
    