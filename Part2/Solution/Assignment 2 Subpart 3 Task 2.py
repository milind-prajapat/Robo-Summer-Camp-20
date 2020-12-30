import pybullet as p
import pybullet_data
import time
p.connect(p.GUI)
p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

C = p.loadURDF("Box.urdf")
p.resetBasePositionAndOrientation(C,[0, 0, 2],[0, 0, 0, 0.707])
p.createConstraint(C, -1, -1, -1, p.JOINT_FIXED,[ 0, 0, 0], [ 0, 0, 0], [ 0, 0, 2])
j = -0.30
for i in range(0,6):
  S = p.loadURDF("Sphere.urdf")
  if(i!=0):
    p.resetBasePositionAndOrientation(S,[j, 0, 1],[0, 0, 0, 0.707])
  else:
    p.resetBasePositionAndOrientation(S,[j-1, 0, 2],[0, 0, 0, 0.707])
  p.createConstraint(C, -1, S, -1, p.JOINT_POINT2POINT,[ 0, 0, 0], [ j, 0, 0], [ 0, 0, 1])
  p.changeDynamics(S,-1,lateralFriction=1.0, spinningFriction=1.0, rollingFriction=1.0, linearDamping=0, angularDamping=0, restitution=0.8)
  j += 0.1

while (1):
  time.sleep(0.01)
  p.stepSimulation()

p.disconnect()
