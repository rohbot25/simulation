import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
from sensor import SENSOR
from motor import MOTOR
class ROBOT:
    def __init__(self):
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
    def Prepare_To_Sense(self):
        self.sensors = dict()
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)
    def Sense(self,t):
        for key in self.sensors:
            self.sensors[key].Get_Value(t)
    def Prepare_To_Act(self):
        self.motors = dict()
        for jointName in pyrosim.jointNamesToIndices:
            print(str(jointName))
            self.motors[jointName] = MOTOR(jointName)
    def Act(self,t):
        for key in self.motors:
            self.motors[key].Set_Value(t,self.robotId)
        

        
            
            
            
        
