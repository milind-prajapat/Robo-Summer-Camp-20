import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt

p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

target_block = p.loadURDF("Block.urdf", [-3, 0, 0.175])
husky = p.loadURDF("husky/husky.urdf", [0, 0, 0])
p.createConstraint(husky, -1, -1, -1, p.JOINT_POINT2POINT, [0, 0, 1], [0, 0, 0], [0, 0, 0])

maxForce = 200
h = 512
w = 512
fov = 60
asp = w/h
near = 0.02
far = 5

Kp = 0.1
Kd = 1
Ki = 0.001
L = []
Last_Error = 0
PID_Control = False
X = 0
Y = 0

def Centre():
    global X
    global Y
    pos, _ = p.getBasePositionAndOrientation(husky)
    L = []
    for i in range(2,6):
      L.append(p.getLinkState(husky,i)[0])
    x = ((L[2][0]+L[3][0])/2 - (L[0][0]+L[1][0])/2)
    y = ((L[2][1]+L[3][1])/2 - (L[0][1]+L[1][1])/2)
    m = round(y/x, 2)
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
    view_matrix = p.computeViewMatrix([i, j, 0.2], [x,y,0.175], [0, 0, 1])
    projection_matrix = p.computeProjectionMatrixFOV(fov, asp, near, far)
    images = p.getCameraImage(w,h,
                              view_matrix,projection_matrix,
                              shadow=True,renderer=p.ER_BULLET_HARDWARE_OPENGL)
    image = np.reshape(images[2], (h,w,4))

    lower = np.array([45,100,50])
    upper = np.array([75,255,255])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(image, image, mask = mask)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 5, 105, cv2.THRESH_BINARY)
    kernel = np.ones((5,5),np.uint8)
    test = cv2.erode(thresh, kernel, iterations = 1)
    test = cv2.dilate(test, kernel, iterations = 1)
    cnts, _ = cv2.findContours(test, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    try:
        M = cv2.moments(cnts[0])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX,cY
    except IndexError:
        print("\n Block Not Visible")
        return X, Y
    
def Turn(Speed):
    baseSpeed = 5
    targetVel_R = baseSpeed + Speed
    targetVel_L = baseSpeed - Speed
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2*joint, p.VELOCITY_CONTROL, targetVelocity =targetVel_R,force = maxForce)
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2*joint+1, p.VELOCITY_CONTROL,targetVelocity =targetVel_L,force = maxForce)
    p.stepSimulation()

X,Y = Centre()

while (1):
    keys = p.getKeyboardEvents()
    if (PID_Control):
        D = I = 0
        x,y = Centre()
        Error = ((X - x)**2 + (Y - y)**2)**(1/2)
        ''' Propotional '''
        if(Last_Error!=0):
            D = Error - Last_Error
            ''' Derivative '''
        L.append(Error)
        if(len(L)>10):
            L.remove(L[0])
            I = np.sum(np.array(L))
            ''' Integral '''
        Speed_Correction = round(Kp*Error + Kd*D + Ki*I,2)
        if x <= X+1 and x >= X-1:
            PID_Control = False
            print("\n PID Control-Off, Back To Manual")
        elif(x>X):
            Turn(Speed_Correction)
        else:
            Turn(-Speed_Correction)
        Last_Error = Error
        
    for k, v in keys.items():
            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN) and PID_Control==False):
                targetVel = 2
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2*joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2*joint+1, p.VELOCITY_CONTROL,targetVelocity =-1*targetVel,force = maxForce)

                p.stepSimulation()
            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)

                p.stepSimulation()
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)and PID_Control==False):
                targetVel = 2
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL,targetVelocity =-1* targetVel,force = maxForce)

                p.stepSimulation()
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)

                p.stepSimulation()
            if (k == ord('c') and (v & p.KEY_WAS_TRIGGERED)):
                print("\n PID Control-On")
                PID_Control = True
    
            if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
                print("\n PID Control-Off, Back To Manual")
                PID_Control = False

p.disconnect()
