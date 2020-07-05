import numpy as np
import math
from numpy.linalg import inv

def get_coord(u,v,d):
    """
    xman, xmin, ymax, and ymin you sould be able to from the detected depth region of the object in the scene.
    u = (xmax-xmin)/2
    y = (ymax-ymin)/2
    :param u: image coordination of the point of the object in x-axis
    :param v: image coordination of the point of the object in y-axis
    :param d: depth information of the point
    :return: world coordinate of the object.
    """
    # K is the intrinsic matrix
    fov = 60
    fx = 1/(math.tan(60/2))
    fy = 1/(math.tan(60/2))
    px = 0
    py = 0
    K = [[fx,0,px],[0,600.688,py],[0,0,1]]
    C = [u, v, d]
    cam = [C[0]*C[2],C[1]*C[2],C[2]]
    kinv = inv(K)
    coordinate = np.matmul(kinv,cam)
    return coordinate