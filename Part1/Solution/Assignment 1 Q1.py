import pybullet as p
import pybullet_data
import os
import time
def Gravity(g):
    p.setGravity(g+0.05,g+0.05,0)
    return(g+0.05)
file_path = os.getcwd() 
p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
p.setGravity(0,0,0)
robot = p.loadURDF(file_path+"/Bot.urdf")
robot1 = p.loadURDF(file_path+"/Box.urdf")
p.resetBasePositionAndOrientation(robot, [2, 2, 1], [0, 0, 0, 0.707])
p.resetBasePositionAndOrientation(robot1, [0, 0, 1], [0, 0, 0, 0.707])
G = 0
while(True):
    p.stepSimulation()
    G = Gravity(G)
    time.sleep(0.01)
    if(G >= 9.8):
        G = 0
        p.resetBasePositionAndOrientation(robot, [2, 2, 1], [0, 0, 0, 0.707])
        p.resetBasePositionAndOrientation(robot1, [0, 0, 1], [0, 0, 0, 0.707])
