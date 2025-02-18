
import pybullet as p
import time
import pybullet_data
import numpy as np
import CreateObjects

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

################################################################################################################
# Loading and setting URDFs

p.loadURDF("plane.urdf", [0, 0, 0.0])
p.setGravity(0, 0, -10)

# Making the maze 
CreateObjects.create_boundaries(physicsClient)

# Placing objects and tray
CreateObjects.create_garbage_and_tray(physicsClient)

# Importing the husky bot 
huskyCenter = [-9, -2, 0.025]#[0.0, 0.0, 0.0]
huskyOrientation = p.getQuaternionFromEuler([0,0,0])
husky = p.loadURDF("husky/husky.urdf", huskyCenter, huskyOrientation)

# Importing kuka arm
kukaCenter = [0.0, 0.0, 0.24023]
kukaOrientation = p.getQuaternionFromEuler([0,0,0])
scale = 0.4
kukaId = p.loadURDF("kuka_experimental-indigo-devel/kuka_kr210_support/urdf/kr210l150.urdf", kukaCenter, kukaOrientation, globalScaling=scale)

# Setting kuka initially 0 
curr_joint_value = [0,0,0,0,0,0,0,0,0,0,0]
p.setJointMotorControlArray(kukaId, range(11), p.POSITION_CONTROL, targetPositions=curr_joint_value)

# Putting kuka on husky
cid = p.createConstraint(husky, 1, kukaId, -1, p.JOINT_FIXED, [0, 0, 0], [0.0, 0.0, 0.14023], [0., 0., 0], [0.0, 0.0, 0.0])

# Activating real time simulation
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)


time.sleep(10)
p.disconnect()

