import pybullet as p
import pybullet_data
import os
file_path = os.getcwd() 
file_name = "/Assignment 1 2D Arm.urdf"
p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
p.setGravity(0,0,0)
robot = p.loadURDF(file_path+file_name)
p.resetBasePositionAndOrientation(robot, [0, 0, 1], [0, 0, 0, 0.707])
while(True):
    p.stepSimulation()
