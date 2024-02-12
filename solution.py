import numpy
import constants as c
import pybullet as p
import pyrosim.pyrosim as pyrosim
import os
import random
import time

class SOLUTION:
    def __init__(self,nextAvailableID):
        self.myID = nextAvailableID
        self.weights = numpy.random.rand(c.numSensorNeurons,c.numMotorNeurons)
        self.weights = self.weights * 2 -1
        

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        #pyrosim.Send_Cube(name="Box", pos=[c.x,c.y,c.z] , size=[c.length,c.width,c.height])
        pyrosim.End()
    
    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[0,0,1] , size=[c.length,c.width,c.height])
        
        pyrosim.Send_Joint( name = "Torso_BackLeg", parent = "Torso", child = "BackLeg", type = "revolute", position = [0,-0.5,1],jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0] , size=[.2,1,.2])
        pyrosim.Send_Joint( name = "BackLeg_BackLowerLeg", parent = "BackLeg", child = "BackLowerLeg", type = "revolute", position = [0,-1,0],jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLowerLeg", pos=[0,0,-.5] , size=[.2,.2,1])
        
        pyrosim.Send_Joint( name = "Torso_FrontLeg", parent = "Torso", child = "FrontLeg", type = "revolute", position = [0,0.5,1],jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0] , size=[.2,1,.2])
        pyrosim.Send_Joint( name = "FrontLeg_FrontLowerLeg", parent = "FrontLeg", child = "FrontLowerLeg", type = "revolute", position = [0,1,0],jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0,0,-.5] , size=[.2,.2,1])

        pyrosim.Send_Joint( name = "Torso_LeftLeg", parent = "Torso", child = "LeftLeg", type = "revolute", position = [-0.5,0,1],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0,0] , size=[1,.2,.2])
        pyrosim.Send_Joint( name = "LeftLeg_LeftLowerLeg", parent = "LeftLeg", child = "LeftLowerLeg", type = "revolute", position = [-1,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0,0,-.5] , size=[.2,.2,1])

        pyrosim.Send_Joint( name = "Torso_RightLeg", parent = "Torso", child = "RightLeg", type = "revolute", position = [0.5,0,1],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0] , size=[1,.2,.2])
        pyrosim.Send_Joint( name = "RightLeg_RightLowerLeg", parent = "RightLeg", child = "RightLowerLeg", type = "revolute", position = [1,0,0],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-.5] , size=[.2,.2,1])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain"+str(self.myID)+".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
        for currentRow in range(c.numSensorNeurons):
            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+c.numSensorNeurons , weight = self.weights[currentRow][currentColumn])
    
        pyrosim.End()
        
    def Mutate(self):
        randomRow = random.randint(0,c.numSensorNeurons-1)
        randomCol = random.randint(0,c.numMotorNeurons-1)
        self.weights[randomRow][randomCol] = random.random() * 2 - 1

    def Set_ID(self,ID):
        self.myID = ID

    def Start_Simulation(self,directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        os.system("python3 simulate.py " + str(directOrGUI) +" "+str(self.myID)+ " &")

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "data/fitness"+str(self.myID)+".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(.01)
        f = open(fitnessFileName,'r')
        self.fitness = float(f.read())
        f.close()
        os.system("rm "+fitnessFileName)
        
        
        
        
