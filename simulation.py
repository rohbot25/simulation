import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import pybullet_data
import time
import numpy
from robot import ROBOT
from world import WORLD
class SIMULATION:
    def __init__(self,directOrGUI):
        if(directOrGUI == "DIRECT"):
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,c.gravity)
        self.world = WORLD()
        self.robot = ROBOT()
        

    def Run(self):
        for i in range(c.iterations):
            #print(i)
            time.sleep(c.sleepy)
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think(i)
            self.robot.Act(i)
    def __del__(self):
        p.disconnect()

    def Get_Fitness(self):
        self.robot.Get_Fitness()

