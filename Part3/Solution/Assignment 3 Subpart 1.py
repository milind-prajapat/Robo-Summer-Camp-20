import pybullet as p
import pybullet_data
import cv2
import numpy as np
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
car = p.loadURDF("husky/husky.urdf", [0,0,0])

numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
  
targetVel = 3  #rad/s
maxForce = 100 #Newton
h = 512
w = 512
fov = 60
asp = w/h
near = 0.02
far = 5

while (1):
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =-targetVel,force = maxForce)
           
            p.stepSimulation()
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =-targetVel,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)

            p.stepSimulation()
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)

            p.stepSimulation()
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =-targetVel,force = maxForce)
           
            p.stepSimulation()
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
          for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)
          for joint in range(3, 6, 2):
            p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)

          p.stepSimulation()
        if (k == ord('a')and (v & p.KEY_WAS_RELEASED)):
            targetVel += 1
            
        if (k == ord('r')and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =-10,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =10,force = maxForce)

            p.stepSimulation()
        if (k == ord('r')and (v & p.KEY_WAS_RELEASED)):
            for joint in range(2, 5, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)
            for joint in range(3, 6, 2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =0,force = maxForce)

            p.stepSimulation()
        if (k == ord('c')and (v & p.KEY_IS_DOWN)):
            pos, _ = p.getBasePositionAndOrientation(car)
            L = []
            for i in range(2,6):
              L.append(p.getLinkState(car,i)[0])
            X = ((L[2][0]+L[3][0])/2 - (L[0][0]+L[1][0])/2)
            Y = ((L[2][1]+L[3][1])/2 - (L[0][1]+L[1][1])/2)
            m = round(Y/X,2)
            C = round((L[0][1]+L[1][1])/2 - m*(L[0][0]+L[1][0])/2,2)
            ''' This Is Equation Of Line Of Symmetry Of The Husky, So As To Get The Co-Ordinates To Move Camera Slighty Forward '''
            a = m**2+1
            b = 2*m*(C-pos[1]) - 2*pos[0]
            c = (pos[0]**2)+((C-pos[1])**2) - 4
            ''' These Are Been Calculated By Applying Distance Formula Between Base Position And The Desired i.e, Required Co-Ordinates '''
            x1 = ((-1)*b-(b**2-4*a*c)**(1/2))/(2*a)
            x2 = ((-1)*b+(b**2-4*a*c)**(1/2))/(2*a)
            y1 = m*x1+C
            y2 = m*x2+C
            d1 = (((L[0][0]+L[1][0])/2 - x1)**2 + ((L[0][1]+L[1][1])/2 - y1)**2)**(1/2)
            d2 = (((L[0][0]+L[1][0])/2 - x2)**2 + ((L[0][1]+L[1][1])/2 - y2)**2)**(1/2)
            if(d1 > d2):
              x = x1
              y = y1
            else:
              x = x2
              y = y2
            ''' This Is The Co-Ordinates Where The Camera Will Be Focusing, As Quadartic Yields Two Co-Ordinates, The Infront Co-Ordinates Are To Be Used '''
            c = (pos[0]**2)+((C-pos[1])**2) - 0.2025
            ''' These Are Been Calculated By Applying Distance Formula Between Base Position And The Desired i.e, Required Co-Ordinates '''
            i1 = ((-1)*b-(b**2-4*a*c)**(1/2))/(2*a)
            i2 = ((-1)*b+(b**2-4*a*c)**(1/2))/(2*a)
            j1 = m*i1+C
            j2 = m*i2+C
            d1 = (((L[0][0]+L[1][0])/2 - i1)**2 + ((L[0][1]+L[1][1])/2 - j1)**2)**(1/2)
            d2 = (((L[0][0]+L[1][0])/2 - i2)**2 + ((L[0][1]+L[1][1])/2 - j2)**2)**(1/2)
            if(d1 > d2):
              i = i1
              j = j1
            else:
              i = i2
              j = j2
            ''' This Is The Co-Ordinates Where The Camera Will Be Placed, As Quadartic Yields Two Co-Ordinates, The Infront Co-Ordinates Are To Be Used'''
            view_matrix = p.computeViewMatrix([i, j, 0.2], [x,y,0.2], [0, 0, 1])
            projection_matrix = p.computeProjectionMatrixFOV(fov, asp, near, far)
            images = p.getCameraImage(w,h,
                                      view_matrix,projection_matrix,
                                      shadow=True,renderer=p.ER_BULLET_HARDWARE_OPENGL)
            image = np.reshape(images[2], (h,w,4))
            cv2.imshow('frame',image)
            cv2.waitKey(0)
         
p.getContactPoints(car)
p.disconnect()
cv2.destroyAllWindows()
