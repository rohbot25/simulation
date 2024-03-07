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
    def Get_Fitness(self,run):
        stateOfLinkZero = p.getLinkState(self.robotId,3)
        linkPosition = stateOfLinkZero[0]
        xPosition = linkPosition[0]
        zPosition = linkPosition[2]
        vfeet = []
        vknee = []
        vhip = []
        afeet = []
        aknee = []
        ahip = []
        #print("fitness: " + str(self.fitnessNums))
        for i in range(len(self.fitnessNums)-1):
            feet = []
            knee = []
            hip = []
            for a in range(len(self.fitnessNums[i])):
                if(a == 1 or a == len(self.fitnessNums[i])-1):
                    feet.append(self.fitnessNums[i+1][a] - self.fitnessNums[i][a])
                elif(a == 2 or a == len(self.fitnessNums[i])-2):
                    knee.append(self.fitnessNums[i+1][a] - self.fitnessNums[i][a])
                else:
                    hip.append(self.fitnessNums[i+1][a] - self.fitnessNums[i][a])
            vfeet.append(feet)
            vknee.append(knee)
            vhip.append(hip)
        #print("Velocity: " + str(velocity))
        for i in range(len(vfeet)-1):
            feet = []
            knee = []
            hip = []
            for a in range(len(vfeet[i])):
                    feet.append(abs(vfeet[i+1][a] - vfeet[i][a]))
                    knee.append(abs(vknee[i+1][a] - vknee[i][a]))
                    hip.append(abs(vhip[i+1][a] - vhip[i][a]))
            afeet.append(feet)
            aknee.append(knee)
            ahip.append(hip)
        f = 0
        n = 0
        h = 0
        for foot in afeet:
            f += sum(foot)
        for knee in aknee:
            n += sum(knee)
        for hip in ahip:
            h += sum(hip)
        #print("Acceleration " + str(acceleration))
        chunk = c.ab / 8
        if(int(run) < chunk):
            print("\n\nA\n\n")
            fitness = xPosition * zPosition
        elif(int(run) < chunk*2):
            print("\n\nB\n\n")
            fitness = xPosition * zPosition * (1/(1+f))
        elif(int(run) < chunk*3):
            print("\n\nC\n\n")
            fitness = xPosition * zPosition * (1/(1+n)) 
        elif(int(run) < chunk*4):
            print("\n\nD\n\n")
            fitness = xPosition * zPosition * (1/(1+h))
        elif(int(run) < chunk*5):
            print("\n\nE\n\n")
            fitness = xPosition * zPosition * (1/(1+f)) * (1/(1+n))
        elif(int(run) < chunk*6):
            print("\n\nF\n\n")
            fitness = xPosition * zPosition * (1/(1+n)) * (1/(1+h))
        elif(int(run) < chunk*7):
            print("\n\nG\n\n")
            fitness = xPosition * zPosition * (1/(1+f)) * (1/(1+h))
        else:
            print("\n\nH\n\n")
            fitness = xPosition * zPosition * (1/(1+f)) * (1/(1+n)) * (1/(1+h))
        
        #fitness = sum(self.fitnessNum)/c.iterations
        tempFile = "data/tmp"+str(self.myID)+".txt"
        xTempFile = "data/tmpx"+str(self.myID)+".txt"
        xFile = "data/x"+str(self.myID)+".txt"
        fitnessFile = "data/fitness"+str(self.myID)+".txt"
        f = open(tempFile,'w')
        x = open(xTempFile,'w')
        
        f.write(str(fitness))
        x.write(str(xPosition))
        
        f.close()
        x.close()
        
        os.system("mv "+tempFile+" "+fitnessFile)
        os.system("mv "+xTempFile+" "+xFile)

        
        

        
            
            
            
        
