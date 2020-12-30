import pybullet as p
import pybullet_data
import time
p.connect(p.GUI)
p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
ramp=p.loadURDF("Wedge.urdf")
p.setGravity(0, 0, -10)
p.changeDynamics(ramp,-1,lateralFriction=0.5)
huskypos = [2, 0, 0.1]
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])

print("Num of Joints",p.getNumJoints(husky))
for i in range(0,p.getNumJoints(husky)):
  print("Joint Info",p.getJointInfo(husky,i))
for i in range(0,p.getNumJoints(husky)):
  print("Joint State",p.getJointState(husky,i))

print("\nSET Control Mode:")
print("1. Torque Mode")
print("2. Velocity Mode")
print("Enter Corresponding Index:")
X = int(input())

if not(X == 1 or X == 2):
  print("Input Error")
  print("Restart Program!!")

def Torque_control():
  optimal_force_value = -250
  for i in range(2,6):
    p.setJointMotorControl2(husky, i,controlMode = p.TORQUE_CONTROL,force = optimal_force_value)
    
def Velocity_control():
  maxForce = 100
  optimal_velocity_value = -20
  for i in range(2,6):
    p.setJointMotorControl2(husky, i, controlMode = p.VELOCITY_CONTROL, targetVelocity = optimal_velocity_value, force = maxForce)

i = 0
while (1):
  if(X == 1):
    Torque_control()
  elif(X == 2):
    Velocity_control()    

  p.stepSimulation()
  if(i%100 == 0):
    print(p.getBasePositionAndOrientation(husky))
    print(p.getBaseVelocity(husky))

p.disconnect()
