import pybullet as p
import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
carpos = [0, 0, 0.1]
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])

numJoints = p.getNumJoints(car)

for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
  
targetVel = 3  #rad/s
maxForce = 100 #Newton

while (1):
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =-targetVel,force = maxForce)
    
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =-targetVel,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =-targetVel,force = maxForce)
           
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
          for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)
          for joint in range(3, 6, 2):
            p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)

        if (k == ord('a')and (v & p.KEY_WAS_RELEASED)):
            targetVel += 1
            
        if (k == ord('r')and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =-10,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =10,force = maxForce)

        if (k == ord('r')and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)

        #pos, _ = p.getBasePositionAndOrientation(car)
        #p.resetDebugVisualizerCamera(5, 0, -20, pos)    
        p.stepSimulation()

p.getContactPoints(car)
p.disconnect()
