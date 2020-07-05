import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid<0):
	p.connect(p.GUI)

p.setGravity(0,0,-9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0, 0, -1])
p.loadURDF("plane.urdf",[0,0,-.98])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf",[0,0,0], [0,0,0,3])  # load sawyer robot
tableId = p.loadURDF("table/table.urdf",[1.4,0, -1], p.getQuaternionFromEuler([0,0,1.56]))   # load table
#cubeId = p.loadURDF("cube/marble_cube.urdf",[1.1,0,0], p.getQuaternionFromEuler([0,0,1.56]))   # load object, change file name to load different objects

######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d])) 
# center of table is at [1.4,0, -1], adjust postion of object to put it on the table                                                          
objectId = p.loadURDF("random_urdfs/108/108.urdf",[1.20,0.0,0.0], p.getQuaternionFromEuler([0,0,1.56]))


text_paths4 = 'Data/dtd/images/bubbly/bubbly_0038.jpg'
text_id4 = p.loadTexture(text_paths4)
p.changeVisualShape(objectId,-1,textureUniqueId=text_id4)

########################################################################################################################################################

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
p.resetBasePositionAndOrientation(sawyerId,[0,0,0],[0,0,0,1])

#bad, get it from name! sawyerEndEffectorIndex = 18
sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)   #65 with ar10 hand
#print(p.getJointInfo(sawyerId, 3))
#useRealTimeSimulation = 0
#p.setRealTimeSimulation(useRealTimeSimulation)
#p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64] 
#lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34, 0.17]
#upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
#joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
#restposes for null space
rp = [0]*35
#joint damping coefficents
jd = [1.1]*35


######################################################### Inverse Kinematics Functions ##########################################################################
#index:51, mid:42, ring: 33, pinky:24, thumb 62

# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion 
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to move to correspond joint position
def palmP(targetP, orientation): 
	jointP = [0]*65
	jointPoses = p.calculateInverseKinematics(sawyerId, 20, targetP, targetOrientation = orientation, jointDamping=jd)
	j=0
	for i in js:
		jointP[i] = jointPoses[j]
		j=j+1
	
	for i in range(p.getNumJoints(sawyerId)):
		
        	p.setJointMotorControl2(bodyIndex=sawyerId, 
					jointIndex=i, 
					controlMode=p.POSITION_CONTROL, 
					targetPosition=jointP[i], 
					targetVelocity=0, 
					force=500, 
					positionGain=0.03, 
					velocityGain=1)
	return jointP

# move hand (fingers) to reach the target postion and orientation
# input: postion and orientation of each fingers
# output: control joint to move to correspond joint position

def handIK(thumbPostion, thumbOrien, indexPostion, indexOrien, midPostion, midOrien, ringPostion, ringOrien, pinkyPostion, pinkyOrien, currentPosition):
	####### thumb IK ##############################################		
	jointP = [0]*65
	joint_thumb = [0]*65
	#lock arm 
	for i in [3, 4, 8, 9, 10, 11, 13, 16]: 
		jointP[i] = currentPosition[i]


	jointPoses_thumb = p.calculateInverseKinematics(sawyerId, 62, thumbPostion, thumbOrien, jointDamping=jd, lowerLimits = ll, 
						  upperLimits = ul, jointRanges=jr, restPoses = currentPosition, 
						  maxNumIterations = 2005, residualThreshold = 0.01)
	# bulit joint position in all joints 65
	j=0
	for i in js: 
		joint_thumb[i] = jointPoses_thumb[j]
		j=j+1	

	# group thumb mid servo
	if(abs(joint_thumb[61] - currentPosition[61]) >= abs(joint_thumb[64] - currentPosition[64])): 
		joint_thumb[64] = joint_thumb[61]	
	else:	
		joint_thumb[61] = joint_thumb[64]

	#build jointP
	for i in [58, 61, 64]:
		jointP[i] = joint_thumb[i]
	

	
	####### index finger IK ######################################	
	joint_index = [0]*65
	jointPoses_index = p.calculateInverseKinematics(sawyerId, 51, indexPostion, indexOrien, jointDamping=jd, lowerLimits = ll, 
						  upperLimits = ul, jointRanges=jr, restPoses = jointP, 
						  maxNumIterations = 2005, residualThreshold = 0.01)
	# bulit joint position in all joints 65
	j=0
	for i in js: 
		joint_index[i] = jointPoses_index[j]
		j=j+1

	# group index low servo
	if(abs(joint_index[48] - joint_thumb[48]) >= abs(joint_index[53] - joint_thumb[53])): 
		joint_index[53] = joint_index[48]	
	else:	
		joint_index[48] = joint_index[53]	
	# group index mid servo
	if(abs(joint_index[49] - joint_thumb[49]) >= abs(joint_index[54] - joint_thumb[54])): 
		joint_index[54] = joint_index[49]	
	else:	
		joint_index[49] = joint_index[54]
	# group index tip servo
	if(abs(joint_index[50] - joint_thumb[50]) >= abs(joint_index[55] - joint_thumb[55])): 
		joint_index[55] = joint_index[50]	
	else:	
		joint_index[50] = joint_index[55]

	#build jointP
	for i in [48, 49, 53, 54]:
		jointP[i] = joint_index[i]


	####### mid finger IK ######################################	

	joint_mid = [0]*65
	jointPoses_mid = p.calculateInverseKinematics(sawyerId, 42, midPostion, midOrien, jointDamping=jd, lowerLimits = ll, 
						  upperLimits = ul, jointRanges=jr, restPoses = jointP, 
						  maxNumIterations = 2005, residualThreshold = 0.01)
	# bulit joint position in all joints 65
	j=0
	for i in js: 
		joint_mid[i] = jointPoses_mid[j]
		j=j+1

	# group mid low servo
	if(abs(joint_mid[39] - joint_index[39]) >= abs(joint_mid[44] - joint_index[44])): 
		joint_mid[44] = joint_mid[39]	
	else:	
		joint_mid[39] = joint_mid[44]	
	# group mid mid servo
	if(abs(joint_mid[40] - joint_index[40]) >= abs(joint_mid[45] - joint_index[45])): 
		joint_mid[45] = joint_mid[40]	
	else:	
		joint_mid[40] = joint_mid[45]
	# group mid tip servo
	if(abs(joint_mid[41] - joint_index[41]) >= abs(joint_mid[46] - joint_index[46])): 
		joint_mid[46] = joint_mid[41]	
	else:	
		joint_mid[41] = joint_mid[46]

	#build jointP
	for i in [39, 40, 44, 45]:
		jointP[i] = joint_mid[i]

	####### ring finger IK ######################################	

	joint_ring = [0]*65
	jointPoses_ring = p.calculateInverseKinematics(sawyerId, 33, ringPostion, ringOrien, jointDamping=jd, lowerLimits = ll, 
						  upperLimits = ul, jointRanges=jr, restPoses = jointP, 
						  maxNumIterations = 2005, residualThreshold = 0.01)
	# bulit joint position in all joints 65
	j=0
	for i in js: 
		joint_ring[i] = jointPoses_ring[j]
		j=j+1
	
	# group ring low servo
	if(abs(joint_ring[30] - joint_mid[30]) >= abs(joint_ring[35] - joint_mid[35])): 
		joint_ring[35] = joint_ring[30]	
	else:	
		joint_ring[30] = joint_ring[35]	
	# group ring mid servo
	if(abs(joint_ring[31] - joint_mid[31]) >= abs(joint_ring[36] - joint_mid[36])): 
		joint_ring[36] = joint_ring[31]	
	else:	
		joint_ring[31] = joint_ring[36]
	# group ring tip servo
	if(abs(joint_ring[32] - joint_mid[32]) >= abs(joint_ring[37] - joint_mid[37])): 
		joint_ring[37] = joint_ring[32]	
	else:	
		joint_ring[32] = joint_ring[37]

	#build jointP
	for i in [30, 31, 35, 36]:
		jointP[i] = joint_ring[i]


	####### pinky finger IK ######################################	

	joint_Pinky = [0]*65
	jointPoses_Pinky = p.calculateInverseKinematics(sawyerId, 24, pinkyPostion, pinkyOrien, jointDamping=jd, lowerLimits = ll, 
						  upperLimits = ul, jointRanges=jr, restPoses = jointP, 
						  maxNumIterations = 2005, residualThreshold = 0.01)
	# bulit joint position in all joints 65
	j=0
	for i in js: 
		joint_Pinky[i] = jointPoses_Pinky[j]
		j=j+1
	
	# group ring low servo
	if(abs(joint_Pinky[21] - joint_ring[21]) >= abs(joint_Pinky[26] - joint_ring[26])): 
		joint_Pinky[26] = joint_Pinky[21]	
	else:	
		joint_Pinky[21] = joint_Pinky[26]	
	# group ring mid servo
	if(abs(joint_Pinky[22] - joint_ring[22]) >= abs(joint_Pinky[27] - joint_ring[27])): 
		joint_Pinky[27] = joint_Pinky[22]	
	else:	
		joint_Pinky[22] = joint_Pinky[27]
	# group ring tip servo
	if(abs(joint_Pinky[23] - joint_ring[23]) >= abs(joint_Pinky[28] - joint_ring[28])): 
		joint_Pinky[28] = joint_Pinky[23]	
	else:	
		joint_Pinky[23] = joint_Pinky[28]

	#build jointP
	for i in [21, 22, 26, 27]:
		jointP[i] = joint_Pinky[i]

	####### move joints ######################################	

	for i in range(p.getNumJoints(sawyerId)): 		
       		p.setJointMotorControl2(bodyIndex=sawyerId, 
					jointIndex=i, 
					controlMode=p.POSITION_CONTROL, 
					targetPosition=jointP[i], 
					targetVelocity=0, 
					force=500, 
					positionGain=0.03,
					velocityGain=1)	

######################################################### Simulation ##########################################################################

# postions and orientations of hand joints, get from project_joint_control.py
thumbP = [1.1625325066735959, -0.10449498868413923, -0.06201584642636722]
thumbOrien = [0.08481418954301742, 0.7830066745269654, -0.5522344438557819, 0.27339389151493354]
indexP = [1.2110404422988899, -0.08285140073825516, -0.05607174103449016]
indexOrien = [0.22403382942495043, 0.8333010224415348, -0.37631931063330054, 0.3373455583160427]
midP = [1.2011953241692468, -0.08000185162493396, -0.07528736347652193]
midOrien = [0.21594704347783641, 0.8420156613112914, -0.35639366389448973, 0.342578541148671]
ringP = [1.1760778497594007, -0.07932664466250612, -0.0817731348857553]
ringOrien = [0.195674772957788, 0.8613029523494512, -0.3068616004516512, 0.3545483967955411]
pinkyP = [1.1407944735130955, -0.08432016557315077, -0.07860648714769963]
pinkyOrien = [0.1377758313070547, 0.8987142263166167, -0.16828415544475644, 0.3808031023377046]

currentP = [0]*65
k = 0
while 1:
	k = k + 1

	# move palm to target postion
	i=0
	while 1:
		i+=1
		p.stepSimulation()
		#currentP = palmP([1.15, -0.07, 0.25], p.getQuaternionFromEuler([0, -math.pi*1.5, 0]))
		#currentP = palmP([2.00, -0.07, 0.25], p.getQuaternionFromEuler([0, -math.pi*1.5, 0]))
		currentP = palmP([1.20,-0.10,0.1], p.getQuaternionFromEuler([0, -math.pi*1.2, 0]))
		time.sleep(0.03)
		if(i==150):
			break

	i=0
	while 1:
		i+=1
		p.stepSimulation()
		#currentP = palmP([1.15, -0.05, -0.15], p.getQuaternionFromEuler([0, -math.pi*1.5, 0]))
		currentP = palmP([1.20,-0.17,-0.3], p.getQuaternionFromEuler([0, -math.pi*1.2, 0]))
		time.sleep(0.03)
		if(i==80):	
			break
	# grasp object
	i=0
	while 1:
		i+=1
		p.stepSimulation()
		time.sleep(0.03)
		handIK(thumbP, thumbOrien, indexP, indexOrien, midP, midOrien, ringP, ringOrien, pinkyP, pinkyOrien, currentP)	
		if(i==80):
			break
	
	# pick up object
	i=0
	while 1:
		i+=1
		p.stepSimulation()
		#currentP = palmP([1.15, -0.07, 0.1], p.getQuaternionFromEuler([0, -math.pi*1.5, 0]))
		currentP = palmP([1.20, -0.20, 0.2], p.getQuaternionFromEuler([0, -math.pi*1.2, 0]))	
	
		time.sleep(0.03)
		if(i==100):
			break
	break

p.disconnect()
print("disconnected")











	



