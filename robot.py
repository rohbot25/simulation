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
        self.fitnessNums = []
        self.robotId = p.loadURDF("body.urdf")
        if(solutionID == '-1'):
            self.nn = NEURAL_NETWORK("best.nndf")
        else:
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
        act = []
        for neuronName in self.nn.Get_Neuron_Names():
            if(self.nn.Is_Motor_Neuron(neuronName)):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                act.append(desiredAngle)
                self.motors[jointName].Set_Value(desiredAngle*c.motorJointRange,self.robotId)
        self.fitnessNums.append(act)
    def Think(self,t):
        self.nn.Update()
        #self.nn.Print()
    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId,2)
        linkPosition = stateOfLinkZero[0]
        xPosition = linkPosition[0]
        zPosition = linkPosition[2]
        velocity = []
        acceleration = []
        jerk = []
        for i in range(len(self.fitnessNums)-1):
            vel = []
            for a in range(len(self.fitnessNums[i])):
                vel.append(self.fitnessNums[i+1][a] - self.fitnessNums[i][a])
            velocity.append(vel)
        for i in range(len(velocity)-1):
            acc = []
            for a in range(len(velocity[i])):
                    acc.append(abs(velocity[i+1][a] - velocity[i][a]))
            acceleration.append(acc)
        for i in range(len(acceleration)-1):
            j = []
            for a in range(len(acceleration[i])-1):
                j.append(abs(acceleration[i+1][a] - acceleration[i][a]))
            jerk.append(j)
        j = 0
        a = 0
        for je in jerk:
            for jerktmp in je:
                if jerktmp > j:
                    j = jerktmp
        for acc in acceleration:
            for atmp in acc:
                if(atmp > a):
                   a = atmp
           
        fitness = xPosition * zPosition
        #* (1/(1+a))
        
        #fitness = sum(self.fitnessNum)/c.iterations
        
        tempFile = "data/tmp"+str(self.myID)+".txt"
        tFile = "data/t"+str(self.myID)+".txt"
        xzFile = "data/xz"+str(self.myID)+".txt"
        fitnessFile = "data/fitness"+str(self.myID)+".txt"
        file = open(tempFile,'w')
        xz = open(tFile,'w')
        file.write(str(fitness))
        xz.write(str(xPosition)+","+str(zPosition)+","+str(a)+","+str(j))
        file.close()
        xz.close()
        os.system("mv "+tempFile+" "+fitnessFile)
        os.system("mv "+tFile+" "+xzFile)
        

        
            
            
            
        
