import pybullet as p
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import constants as c
from sensor import SENSOR
from motor import MOTOR
import os
import numpy

class ROBOT:
    def __init__(self,solutionID):
        self.myID = solutionID
        self.robotId = p.loadURDF("body.urdf")
        self.nn = NEURAL_NETWORK("brain"+str(solutionID)+".nndf")
        os.system("rm brain"+str(self.myID)+".nndf")
        self.fitnessNums = []
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
        act = []
        for neuronName in self.nn.Get_Neuron_Names():
            if(self.nn.Is_Motor_Neuron(neuronName)):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                act.append(desiredAngle)
                self.motors[jointName].Set_Value(desiredAngle,self.robotId)
        self.fitnessNums.append(act)      
    def Think(self,t):
        self.nn.Update()
        #self.nn.Print()
    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId,3)
        linkPosition = stateOfLinkZero[0]
        xPosition = linkPosition[0]
        zPosition = linkPosition[2]
        velocity = []
        acceleration = []
        #print("fitness: " + str(self.fitnessNums))
        for i in range(len(self.fitnessNums)-1):
            vel = []
            for a in range(len(self.fitnessNums[i])):
                vel.append(self.fitnessNums[i+1][a] - self.fitnessNums[i][a])
            velocity.append(vel)
        #print("Velocity: " + str(velocity))
        for i in range(len(velocity)-1):
            acc = []
            for a in range(len(velocity[i])):
                acc.append(abs(velocity[i+1][a] - velocity[i][a]))
            acceleration.append(acc)
        a = 0
        for lists in acceleration:
           a += sum(lists) 
        #print("Acceleration " + str(acceleration))
        fitness = xPosition * zPosition * 1/(1+a)
        #fitness = sum(self.fitnessNum)/c.iterations
        tempFile = "data/tmp"+str(self.myID)+".txt"
        fitnessFile = "data/fitness"+str(self.myID)+".txt"
        f = open(tempFile,'w')
        f.write(str(fitness))
        f.close()
        os.system("mv "+tempFile+" "+fitnessFile)

        
        

        
            
            
            
        
