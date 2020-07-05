import pybullet as p
import time
import math
import random
import numpy as np
import pybullet_data
import random
import glob
import os
import matplotlib.pyplot as plt
from datetime import datetime

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

# p.setGravity(0,0,-9.8
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_path = 'Data/plane/plane.urdf'
planeId = p.loadURDF("plane.urdf", [0, 0, -1])

# p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# the path of robot urdf, should be under the folder ../Data/sawyer_robot/sawyer_description/urdf/sawyer.urdf
robot_path = 'Data/sawyer_robot/sawyer_description/urdf/sawyer.urdf'
sawyerId = p.loadURDF(robot_path, [0, 0, 0],
                      [0, 0, 0, 3])
# the path of the table, should be under the folder ../Data/table/table.urdf
table_path = 'Data/table/table.urdf'
tableId = p.loadURDF(table_path, [1.4, 0, -1], p.getQuaternionFromEuler([0, 0, 1.56]))  # load table


######################################################### Load Object Here!!!!#############################################################################

# load the assigned objects, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# center of table is at [1.4,0, -1], adjust postion of object to put it on the table

# the path of the objects, should be under Data/random_urdfs/000/000.urdf
# Example
#object_1_path = 'Data/random_urdfs/105/105.urdf'
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.004, 0], p.getQuaternionFromEuler([0, 0, 1.56]))

#objectId1 = p.loadURDF(object_1_path, [1, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1, 0.004, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.03, 0.0045, 0.04], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.03, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.03, 0.0055, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.03, 0.0060, 0.04], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.03, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.03, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.03, 0.0075, 0.06], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.06, 0.0030, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.06, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.06, 0.004, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.06, 0.0045, 0.06], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.06, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.06, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1, 0.1, -0.1], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.06, 0.0065, 0.03], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.06, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.06, 0.0075, 0.08], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.030, 0.07], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.004, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.0045, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.0060, 0.06], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.09, 0.08], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.1, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.004, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.0045, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.0060, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.14, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.004, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.0045, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.0060, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))
#objectId1 = p.loadURDF(object_1_path, [1.18, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.56]))

object_2_path = 'Data/random_urdfs/106/106.urdf'
#objectId2 = p.loadURDF(object_2_path, [1.2, 0.2, 0], p.getQuaternionFromEuler([0, 0, 1.57]))

#objectId2 = p.loadURDF(object_2_path, [1.03, 0.10, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #1
#objectId2 = p.loadURDF(object_2_path, [1.03, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #2
#objectId2 = p.loadURDF(object_2_path, [1.03, 0.004, 0.05], p.getQuaternionFromEuler([0, 0, 1.57]))  #3
#objectId2 = p.loadURDF(object_2_path, [1.03, 0.0045, 0.04], p.getQuaternionFromEuler([0, 0, 1.57])) #4 
#objectId2 = p.loadURDF(object_2_path, [1.03, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.57]))     #5
#objectId2 = p.loadURDF(object_2_path, [1.03, 0.0055, 0.05], p.getQuaternionFromEuler([0, 0, 1.57])) #6
#objectId2 = p.loadURDF(object_2_path, [1.03, 0.0060, 0.04], p.getQuaternionFromEuler([0, 0, 1.57])) #7
#objectId2 = p.loadURDF(object_2_path, [1.03, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #8
#objectId2 = p.loadURDF(object_2_path, [1.03, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #9
#objectId2 = p.loadURDF(object_2_path, [1.03, 0.0075, 0.06], p.getQuaternionFromEuler([0, 0, 1.57])) #10
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.0030, 0.02], p.getQuaternionFromEuler([0, 0, 1.57])) #11
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #12
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.004, 0.01], p.getQuaternionFromEuler([0, 0, 1.57]))  #13
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.0045, 0.06], p.getQuaternionFromEuler([0, 0, 1.57])) #14 
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.57]))     #15
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #16
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.0060, 0.05], p.getQuaternionFromEuler([0, 0, 1.57])) #17
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.0065, 0.03], p.getQuaternionFromEuler([0, 0, 1.57])) #18
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #19
#objectId2 = p.loadURDF(object_2_path, [1.06, 0.0075, 0.08], p.getQuaternionFromEuler([0, 0, 1.57])) #20
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.57]))     #21
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.0025, 0], p.getQuaternionFromEuler([0, 0, 1.57]))     #22
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.0020, 0], p.getQuaternionFromEuler([0, 0, 1.57]))   #23
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.004, 0.05], p.getQuaternionFromEuler([0, 0, 1.57]))  #24
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.57]))        #25
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.57]))   #26
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.0060, 0.06], p.getQuaternionFromEuler([0, 0, 1.57]))  #27
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.0065, 0.08], p.getQuaternionFromEuler([0, 0, 1.57]))  #28
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.57]))   #29
#objectId2 = p.loadURDF(object_2_path, [1.1, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.57]))   #30
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.57]))   #31
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.57]))   #32
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.004, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #33
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.0045, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #34
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #35
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.57]))   #36
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.0060, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #37
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #38
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #39
#objectId2 = p.loadURDF(object_2_path, [1.14, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #40
#objectId2 = p.loadURDF(object_2_path, [1.18, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #41
#objectId2 = p.loadURDF(object_2_path, [1.18, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #42
#objectId2 = p.loadURDF(object_2_path, [1.18, 0.004, 0], p.getQuaternionFromEuler([0, 0, 1.57]))     #43
#objectId2 = p.loadURDF(object_2_path, [1.18, 0.0045, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #44
#objectId2 = p.loadURDF(object_2_path, [1.18, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.57]))     #45
#objectId2 = p.loadURDF(object_2_path, [1.18, 0.0020, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #46
#objectId2 = p.loadURDF(object_2_path, [1.18, 0.0024, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #47
#objectId2 = p.loadURDF(object_2_path, [1.18, 0.0026, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #48
#objectId2 = p.loadURDF(object_2_path, [1.18, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #49
objectId2 = p.loadURDF(object_2_path, [1.18, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.57]))    #50


object_3_path = 'Data/random_urdfs/107/107.urdf'
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.1, 0], p.getQuaternionFromEuler([0, 0, 1.56]))

#objectId3 = p.loadURDF(object_3_path, [1.03, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #1
#objectId3 = p.loadURDF(object_3_path, [1.07, 0.070, 0.08], p.getQuaternionFromEuler([0, 0, 1.56]))    #2
#objectId3 = p.loadURDF(object_3_path, [1.03, 0.004, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))  #3
#objectId3 = p.loadURDF(object_3_path, [1.03, 0.0045, 0.04], p.getQuaternionFromEuler([0, 0, 1.56])) #4 
#objectId3 = p.loadURDF(object_3_path, [1.03, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #5
#objectId3 = p.loadURDF(object_3_path, [1.03, 0.0055, 0.05], p.getQuaternionFromEuler([0, 0, 1.56])) #6
#objectId3 = p.loadURDF(object_3_path, [1.03, 0.0060, 0.04], p.getQuaternionFromEuler([0, 0, 1.56])) #7
#objectId3 = p.loadURDF(object_3_path, [1.03, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #8
#objectId3 = p.loadURDF(object_3_path, [1.03, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #9
#objectId3 = p.loadURDF(object_3_path, [1.03, 0.0075, 0.06], p.getQuaternionFromEuler([0, 0, 1.56])) #10
objectId3 = p.loadURDF(object_3_path, [1.03, 0.0030, 0.02], p.getQuaternionFromEuler([0, 0, 1.56])) #11
#objectId3 = p.loadURDF(object_3_path, [1.06, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #12
#objectId3 = p.loadURDF(object_3_path, [1.06, 0.004, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))  #13
#objectId3 = p.loadURDF(object_3_path, [1.06, 0.0045, 0.06], p.getQuaternionFromEuler([0, 0, 1.56])) #14 
#objectId3 = p.loadURDF(object_3_path, [1.06, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #15
#objectId3 = p.loadURDF(object_3_path, [1.06, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #16
#objectId3 = p.loadURDF(object_3_path, [1.06, 0.0060, 0.05], p.getQuaternionFromEuler([0, 0, 1.56])) #17
#objectId3 = p.loadURDF(object_3_path, [1.06, 0.0065, 0.03], p.getQuaternionFromEuler([0, 0, 1.56])) #18
#objectId3 = p.loadURDF(object_3_path, [1.06, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #19
#objectId3 = p.loadURDF(object_3_path, [1.06, 0.0075, 0.08], p.getQuaternionFromEuler([0, 0, 1.56])) #20
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #21
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.0025, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #22
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.0020, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #23
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.004, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))  #24
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))        #25
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #26
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.0060, 0.06], p.getQuaternionFromEuler([0, 0, 1.56]))  #27
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.0065, 0.08], p.getQuaternionFromEuler([0, 0, 1.56]))  #28
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #29
#objectId3 = p.loadURDF(object_3_path, [1.1, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #30
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #31
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #32
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.004, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #33
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.0045, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #34
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #35
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #36
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.0060, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #37
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #38
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #39
#objectId3 = p.loadURDF(object_3_path, [1.14, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #40
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.0030, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))    #41
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.0035, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #42
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.004, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))     #43
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.0045, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #44
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.005, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))     #45
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.0020, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #46
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.0024, 0.07], p.getQuaternionFromEuler([0, 0, 1.56]))    #47
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.0026, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))    #48
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.0070, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #49
#objectId3 = p.loadURDF(object_3_path, [1.12, 0.0075, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))    #50


object_4_path = 'Data/random_urdfs/108/108.urdf'
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.1, 0], p.getQuaternionFromEuler([0, 0, 1.56]))

#objectId4 = p.loadURDF(object_4_path, [1.03, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #1
#objectId4 = p.loadURDF(object_4_path, [1.03, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #2
#objectId4 = p.loadURDF(object_4_path, [1.03, 0.004, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))  #3
#objectId4 = p.loadURDF(object_4_path, [1.03, 0.0045, 0.04], p.getQuaternionFromEuler([0, 0, 1.56])) #4 
objectId4 = p.loadURDF(object_4_path, [1.1, -0.09, 0.08], p.getQuaternionFromEuler([0, 0, 1.56]))     #5
#objectId4 = p.loadURDF(object_4_path, [1.03, 0.0055, 0.05], p.getQuaternionFromEuler([0, 0, 1.56])) #6
#objectId4 = p.loadURDF(object_4_path, [1.03, 0.0060, 0.04], p.getQuaternionFromEuler([0, 0, 1.56])) #7
#objectId4 = p.loadURDF(object_4_path, [1.03, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #8
#objectId4 = p.loadURDF(object_4_path, [1.03, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #9
#objectId4 = p.loadURDF(object_4_path, [1.03, 0.0075, 0.06], p.getQuaternionFromEuler([0, 0, 1.56])) #10
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.0030, 0.02], p.getQuaternionFromEuler([0, 0, 1.56])) #11
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #12
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.004, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))  #13
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.0045, 0.06], p.getQuaternionFromEuler([0, 0, 1.56])) #14 
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #15
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #16
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.0060, 0.05], p.getQuaternionFromEuler([0, 0, 1.56])) #17
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.0065, 0.03], p.getQuaternionFromEuler([0, 0, 1.56])) #18
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #19
#objectId4 = p.loadURDF(object_4_path, [1.06, 0.0075, 0.08], p.getQuaternionFromEuler([0, 0, 1.56])) #20
#objectId4 = p.loadURDF(object_4_path, [1.03, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #21
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.0025, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #22
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.0020, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #23
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.004, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))  #24
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))        #25
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #26
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.0060, 0.06], p.getQuaternionFromEuler([0, 0, 1.56]))  #27
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.0065, 0.08], p.getQuaternionFromEuler([0, 0, 1.56]))  #28
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #29
#objectId4 = p.loadURDF(object_4_path, [1.1, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #30
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.070, 0.09], p.getQuaternionFromEuler([0, 0, 1.56]))   #31
#objectId4 = p.loadURDF(object_4_path, [1.14, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #32
#objectId4 = p.loadURDF(object_4_path, [1.14, 0.004, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #33
#objectId4 = p.loadURDF(object_4_path, [1.14, 0.0045, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #34
#objectId4 = p.loadURDF(object_4_path, [1.14, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #35
#objectId4 = p.loadURDF(object_4_path, [1.14, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #36
#objectId4 = p.loadURDF(object_4_path, [1.14, 0.0060, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #37
#objectId4 = p.loadURDF(object_4_path, [1.14, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #38
#objectId4 = p.loadURDF(object_4_path, [1.14, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #39
#objectId4 = p.loadURDF(object_4_path, [1.14, 0.00075, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #40
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.0030, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))    #41
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.0035, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #42
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.004, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))     #43
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.0045, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #44
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.005, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))     #45
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.0020, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #46
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.0024, 0.07], p.getQuaternionFromEuler([0, 0, 1.56]))    #47
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.0026, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))    #48
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.0070, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #49
#objectId4 = p.loadURDF(object_4_path, [1.12, 0.0075, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))    #50


object_5_path = 'Data/random_urdfs/109/109.urdf'
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.1, 0], p.getQuaternionFromEuler([0, 0, 1.56]))

#objectId5 = p.loadURDF(object_5_path, [1.02, 0.05, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #1
#objectId5 = p.loadURDF(object_5_path, [1, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #2
#objectId5 = p.loadURDF(object_5_path, [1.03, 0.004, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))  #3
#objectId5 = p.loadURDF(object_5_path, [1.03, 0.0045, 0.04], p.getQuaternionFromEuler([0, 0, 1.56])) #4 
#objectId5 = p.loadURDF(object_5_path, [1, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #5
#objectId5 = p.loadURDF(object_5_path, [1.03, 0.0055, 0.05], p.getQuaternionFromEuler([0, 0, 1.56])) #6
#objectId5 = p.loadURDF(object_5_path, [1.03, 0.0060, 0.04], p.getQuaternionFromEuler([0, 0, 1.56])) #7
#objectId5 = p.loadURDF(object_5_path, [1.03, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #8
#objectId5 = p.loadURDF(object_5_path, [1.03, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #9
#objectId5 = p.loadURDF(object_5_path, [1.03, 0.0075, 0.06], p.getQuaternionFromEuler([0, 0, 1.56])) #10
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.0030, 0.02], p.getQuaternionFromEuler([0, 0, 1.56])) #11
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #12
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.004, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))  #13
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.0045, 0.06], p.getQuaternionFromEuler([0, 0, 1.56])) #14 
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #15
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #16
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.0060, 0.05], p.getQuaternionFromEuler([0, 0, 1.56])) #17
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.0065, 0.03], p.getQuaternionFromEuler([0, 0, 1.56])) #18
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #19
#objectId5 = p.loadURDF(object_5_path, [1.06, 0.0075, 0.08], p.getQuaternionFromEuler([0, 0, 1.56])) #20
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.0030, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #21
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.0025, 0], p.getQuaternionFromEuler([0, 0, 1.56]))     #22
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.0020, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #23
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.004, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))  #24
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))        #25
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #26
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.0060, 0.06], p.getQuaternionFromEuler([0, 0, 1.56]))  #27
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.0065, 0.08], p.getQuaternionFromEuler([0, 0, 1.56]))  #28
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #29
#objectId5 = p.loadURDF(object_5_path, [1.1, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #30
objectId5 = p.loadURDF(object_5_path, [1.03, 0.1, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #31
#objectId5 = p.loadURDF(object_5_path, [1.14, 0.0035, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #32
#objectId5 = p.loadURDF(object_5_path, [1.14, -0.1, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #33
#objectId5 = p.loadURDF(object_5_path, [1.14, 0.0045, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #34
#objectId5 = p.loadURDF(object_5_path, [1.14, 0.005, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #35
#objectId5 = p.loadURDF(object_5_path, [1.14, 0.0055, 0], p.getQuaternionFromEuler([0, 0, 1.56]))   #36
#objectId5 = p.loadURDF(object_5_path, [1.14, 0.0060, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #37
#objectId5 = p.loadURDF(object_5_path, [1.14, 0.0065, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #38
#objectId5 = p.loadURDF(object_5_path, [1.14, 0.0070, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #39
#objectId5 = p.loadURDF(object_5_path, [1.14, 0.0075, 0], p.getQuaternionFromEuler([0, 0, 1.56]))    #40
#objectId5 = p.loadURDF(object_5_path, [1.12, 0.0030, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))    #41
#objectId5 = p.loadURDF(object_5_path, [1.12, 0.0035, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #42
#objectId5 = p.loadURDF(object_5_path, [1.12, 0.004, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))     #43
#objectId5 = p.loadURDF(object_5_path, [1.12, 0.0045, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #44
#objectId5 = p.loadURDF(object_5_path, [1.12, 0.005, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))     #45
#objectId5 = p.loadURDF(object_5_path, [1.12, 0.0020, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #46
#objectId5 = p.loadURDF(object_5_path, [1.12, 0.0024, 0.07], p.getQuaternionFromEuler([0, 0, 1.56]))    #47
#objectId5 = p.loadURDF(object_5_path, [1.12, 0.0026, 0.05], p.getQuaternionFromEuler([0, 0, 1.56]))    #48
#objectId5 = p.loadURDF(object_5_path, [1.12, 0.0070, 0.02], p.getQuaternionFromEuler([0, 0, 1.56]))    #49
#objectId5 = p.loadURDF(object_5_path, [1.16, 0.0075, 0.01], p.getQuaternionFromEuler([0, 0, 1.56]))    #50



# apply texture to objects
# apply randomly textures from the dtd dataset to each object, each object's texture should be the different.

# texture_paths = Data/dtd/images/banded/banded_0002.jpg
# example:
#text_paths1 = 'Data/dtd/images/banded/banded_0002.jpg'
#text_id1 = p.loadTexture(text_paths1)
#p.changeVisualShape(objectId1,-1,textureUniqueId=text_id1)
text_paths2 = 'Data/dtd/images/blotchy/blotchy_0003.jpg'
text_id2 = p.loadTexture(text_paths2)
p.changeVisualShape(objectId2,-1,textureUniqueId=text_id2)
text_paths3 = 'Data/dtd/images/braided/braided_0002.jpg'
text_id3 = p.loadTexture(text_paths3)
p.changeVisualShape(objectId3,-1,textureUniqueId=text_id3)
text_paths4 = 'Data/dtd/images/bubbly/bubbly_0038.jpg'
text_id4 = p.loadTexture(text_paths4)
p.changeVisualShape(objectId4,-1,textureUniqueId=text_id4)
text_paths5 = 'Data/dtd/images/bumpy/bumpy_0014.jpg'
text_id5 = p.loadTexture(text_paths5)
p.changeVisualShape(objectId5,-1,textureUniqueId=text_id5)


########################################################Insert Camera####################################################################################
# Using the inserted camera to caputure data for training. Save the captured numpy array as image files for later training process.

width = 256
height = 256

fov = 60
aspect = width / height
near = 0.02
far = 1

# the view_matrix should contain three arguments, the first one is the [X,Y,Z] for camera location
#												  the second one is the [X,Y,Z] for target location
#											      the third one is the  [X,Y,Z] for the top of the camera
# Example:
# viewMatrix = pb.computeViewMatrix(
#     cameraEyePosition=[0, 0, 3],
#     cameraTargetPosition=[0, 0, 0],
#     cameraUpVector=[0, 1, 0])
view_matrix = p.computeViewMatrix([1.15, -0.07, 0.25], [1.1, 0, 0], [-1, 0, 0])

projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)


# Get depth values using the OpenGL renderer
images = p.getCameraImage(width,
                          height,
                          view_matrix,
                          projection_matrix,
                          shadow=True,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
depth_buffer_opengl = np.reshape(images[3], [width, height])
depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
plt.imshow(rgb_opengl)
plt.savefig('Data/object6images/obj100.png')
time.sleep(1)


########################################################################################################################################################

# p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

# bad, get it from name! sawyerEndEffectorIndex = 18
sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand
# print(p.getJointInfo(sawyerId, 3))
# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
      53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1377, -3.8183, -3.0513, -3.0513, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
      0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
      0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0513, 3.0513, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
      0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
      0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35

i = 0
while 1:
    i += 1
    p.stepSimulation()
    # 0.03 sec/frame
    time.sleep(0.03)
    # increase i to incease the simulation time
    if (i == 1800):
        break