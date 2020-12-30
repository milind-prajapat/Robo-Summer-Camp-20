import pybullet as p
import pybullet_data
import os
import time
L = []
def Fab(a,b):
        c=a+b
        for i in range(len(L),c):
                robot = p.loadURDF(file_path+file_name)
                L.append(robot)
        j=0
        for i in range(0,len(L)):
                p.resetBasePositionAndOrientation(L[i], [j, 0, 4], [0, 0, 0, 0.707])
                j+=0.2
        return(c)
file_path = os.getcwd() 
file_name = "/Sphere.urdf"
p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
p.setGravity(0,0,-10)
x = 0
y = 1
while(True):
        temp = Fab(x,y)
        x = y
        y = temp
        for i in range(0,300):
                p.stepSimulation()
                time.sleep(0.000001)
