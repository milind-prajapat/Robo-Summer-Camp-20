import pybullet as p
import pybullet_data
import os
import time
import math
file_name = "2R_planar_robot.urdf"
p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_name)
p.resetBasePositionAndOrientation(robot, [0, 0, 0.05], [0, 0, 0, 0.707])
p.setGravity(0,0,-10)
l1 = 1 
l2 = 1
f = -1
''' Initially f = -1, As Angle is Negative For +ve Y Axis And Positive for -ve Y Axis In PyBullet World's Frame and arccos Only Yields a +ve Angle As Per Its Range [0,3.14] '''
fact = -1
''' Initailly fact = -1 As Target Co-ordinate Needs To Be Decreased First... Then Needed To Be Increased After It Reaches End Point '''
def Draw_Line(point_A,point_B):
        p.addUserDebugLine(point_A,point_B,[1,0,0],4) 

def Forward_Kinematics(angle_1,angle_2):
        y = round(l1*math.cos(angle_1) + l2*math.cos(angle_1 + angle_2))
        z = round(l1*math.sin(angle_1) + l2*math.sin(angle_1 + angle_2))
        ''' No Need To Change Angles To PyBullet World's Frame As This Calculation Should Be Done With User World's Frame And The Points Does Not Changes In Two Different Worlds  '''
        return [0,y,z]

def Inverse_Kinematics(target):
        global f
        cos = (target[1]**2 + target[2]**2 - l1**2 - l2**2)/(2*l1*l2)
        angle_2 = f*math.acos(cos)
        ''' Multiplied by f to Reverse The Angle Sign As Mentioned Earlier ''' 
        a = (l1**2)+(2*cos*l1*l2)+(l2**2)
        b = (-1)*2*target[2]*(l1+cos*l2)
        c = target[2]**2+((l2**2)*(cos**2 -1))
        try:
                sin = ((-1)*b - (b**2 - 4*a*c)**(1/2))/(2*a)
                cos = (1 - sin**2)**(1/2)
                ''' Changed Sin to Cos as Range of Cos is [0,3.14], it Makes it Easy to Make Relation in PyBullet World's Frame Angles And User World Angles '''
                angle_1 = (1.57 - math.acos(cos))*f
                ''' Multiplied by f to Reverse The Angle Sign As Mentioned Earlier ''' 
        except ZeroDivisionError:
                try:
                        sin = c / b
                        cos = (1 - sin**2)**(1/2)
                        ''' Changed Sin to Cos as Range of Cos is [0,3.14], it Makes it Easy to Make Relation in PyBullet World's Frame Angles And User World Angles '''
                        angle_1 = (1.57 - math.acos(cos))*f
                        ''' Multiplied by f to Reverse The Angle Sign As Mentioned Earlier ''' 
                except ZeroDivisionError:
                        angle_1 = 0
                        ''' When it Passes the Axis i.e., The Origin, angle_1 Cannot be Calculated Using This Method, Which Is Indeed Zero At That Point In PyBullet World's Frame '''
                        f *= -1
                        ''' f Changes Sign, As Angles Change Sign As Well in PyBullet World's Frame If It Crosses The Origin i.e., For -ve Y Axis Angles Should Be +ve '''
        return angle_1,angle_2

def Simulate(i,angle_1,angle_2):
        for _ in range(i):
                p.setJointMotorControl2(bodyIndex=robot,
                                        jointIndex=0,
                                        controlMode =p.POSITION_CONTROL,
                                        targetPosition=angle_1,
                                        force=500)

                p.setJointMotorControl2(bodyIndex=robot,
                                        jointIndex=1,
                                        controlMode =p.POSITION_CONTROL,
                                        targetPosition=angle_2,
                                        force=500)
                p.stepSimulation()
                time.sleep(0.01)

def Longest_Line():
        Point_A = Forward_Kinematics(0,0)
        Point_B = Forward_Kinematics(3.14,0)
        Draw_Line(Point_A,Point_B)
        return(Point_A,Point_B)

def Set_At_Point(i,target):
        angle_1,angle_2  = Inverse_Kinematics(target)
        Simulate(i,angle_1,angle_2)
        
A ,B = Longest_Line()
y = A[1]
z = A[2]
Set_At_Point(40, A)

try:
        ''' Equation of the type z = my + C '''
        m = (B[2]-A[2])/(B[1]-A[1])
        C = (-1)*m*A[1] + A[2]
        eq = "Type_1"
        if(C>=0):
                print("z = ",m," y + ",C)
        else:
                print("z = ",m," y ",C)
except ZeroDivisionError:
        ''' Equation of the type y = mz + C '''
        m = (B[1]-A[1])/(B[2]-A[2])
        C = (-1)*m*A[2] + A[1]
        eq = "Type_2"
        if(C>=0):
                print("y = ",m," z + ",C)
        else:
                print("y = ",m," z ",C)       
while(True):
        if(eq == "Type_1"):
                y += 0.01*fact
                z = m*y + C
        elif(eq == "Type_2"):
                z += 0.01*fact
                y = m*z + C
        y = round(y,2)
        z = round(z,2)
        ''' Rounding Off To Gain Accuracy While Substracting Floats, In Python Substracting Floats Generates A Truncated Non Accurate Value '''
        if (y <= 0.05)and(y >= -0.05)and(z <= 0.05)and(z >= -0.05):
                Set_At_Point(15,[0,y,z])
                ''' To Make Simulation Visible, a 360 Turn Is To Be Take Here '''
        else:
                Set_At_Point(1,[0,y,z])
        if ((y <= B[1])and(z <= B[2]))or((y >= A[1])and(z >= A[2])):
                ''' Adding 0.01 to End Point Check Condition As Float Sunstraction In Python Is Not Accurate '''
                fact *= -1
                ''' Sign Needs To be Changed To Move Over Line From End To End '''
