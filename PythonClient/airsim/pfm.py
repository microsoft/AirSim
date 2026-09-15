import numpy as np
import re
import sys


def read_pfm(file):
    """Read a pfm file. `file` is a path (str/Path). Returns (data, scale)."""
    with open(file, 'rb') as f:
        color = None
        width = None
        height = None
        scale = None
        endian = None

        header = f.readline().rstrip()
        header = str(bytes.decode(header, encoding='utf-8'))
        if header == 'PF':
            color = True
        elif header == 'Pf':
            color = False
        else:
            raise Exception('Not a PFM file.')

        pattern = r'^(\d+)\s(\d+)\s$'
        temp_str = str(bytes.decode(f.readline(), encoding='utf-8'))
        dim_match = re.match(pattern, temp_str)
        if dim_match:
            width, height = map(int, dim_match.groups())
        else:
            temp_str += str(bytes.decode(f.readline(), encoding='utf-8'))
            dim_match = re.match(pattern, temp_str)
            if dim_match:
                width, height = map(int, dim_match.groups())
            else:
                raise Exception('Malformed PFM header: width, height cannot be found')

        scale = float(f.readline().rstrip())
        if scale < 0:  # little-endian
            endian = '<'
            scale = -scale
        else:
            endian = '>'  # big-endian

        data = np.fromfile(f, endian + 'f')
        shape = (height, width, 3) if color else (height, width)
        data = np.reshape(data, shape)

    return data, scale


def write_pfm(file, image, scale=1):
    """Write a pfm file. `file` is a path (str/Path)."""
    color = None

    if image.dtype.name != 'float32':
        raise Exception('Image dtype must be float32.')

    if len(image.shape) == 3 and image.shape[2] == 3:  # color image
        color = True
    elif len(image.shape) == 2 or len(image.shape) == 3 and image.shape[2] == 1:  # greyscale
        color = False
    else:
        raise Exception('Image must have H x W x 3, H x W x 1 or H x W dimensions.')

    with open(file, 'wb') as f:
        f.write(bytes('PF\n', 'UTF-8') if color else bytes('Pf\n', 'UTF-8'))
        temp_str = '%d %d\n' % (image.shape[1], image.shape[0])
        f.write(bytes(temp_str, 'UTF-8'))

        endian = image.dtype.byteorder

        if endian == '<' or endian == '=' and sys.byteorder == 'little':
            scale = -scale

        temp_str = '%f\n' % scale
        f.write(bytes(temp_str, 'UTF-8'))

        image.tofile(f)
