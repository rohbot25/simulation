import pybullet as p
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import constants as c
from sensor import SENSOR
from motor import MOTOR
import os

class ROBOT:
    def __init__(self,solutionID):
        self.myID = solutionID
        self.robotId = p.loadURDF("body.urdf")
        self.nn = NEURAL_NETWORK("brain"+str(solutionID)+".nndf")
        os.system("rm brain"+str(self.myID)+".nndf")
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
            self.motors[jointName.decode('ASCII')] = MOTOR(jointName)
    def Act(self,desiredAngle):
        for neuronName in self.nn.Get_Neuron_Names():
            if(self.nn.Is_Motor_Neuron(neuronName)):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[jointName].Set_Value(desiredAngle*c.motorJointRange,self.robotId)
    def Think(self,t):
        self.nn.Update()
        #self.nn.Print()
    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        tempFile = "data/tmp"+str(self.myID)+".txt"
        fitnessFile = "data/fitness"+str(self.myID)+".txt"
        f = open(tempFile,'w')
        f.write(str(xCoordinateOfLinkZero))
        f.close()
        os.system("mv "+tempFile+" "+fitnessFile)
        

        
            
            
            
        
