import numpy as np
import matplotlib.pyplot as plt



def get_center(xmax, ymax, xmin, ymin, depth_path):
    """
    xmax: is the xmax of the detected region
    ymax: is the ymax of the detected region
    xmin: is hte xmin of the detected region
    ymin: is hte ymin of hte detected region
    depth_path is the location of the depth image and its demension should be w*h (eg. 256*256)
    """
    depthImage = plt.imread(depth_path)
    # because the depth image and rgb image are in the same coordination. The boudning box of the
    # rgb image should be in the same region of the depth image.
    depth_region = depthImage[xmin:xmax,ymin:ymax]
    print(depth_region)
    # find the mean of the depth region use as the representative depth
    d = depth_region.mean()

    # find the image coordination of the bounding box region.
    u = (xmax-xmin)/2
    v = (ymax-ymin)/2

    return d, u, v



d, u, v= get_center(140,160,105,100,'depth_0.png')
print(d)
print(u)
print(v)




