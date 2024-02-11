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
        self.weights = numpy.random.rand(3,2)
        self.weights = self.weights * 2 -1
        
    def Evaluate(self,directOrGUI):
        self.Start_Simulation(directOrGUI)

        self.Wait_For_Simulation_To_End()

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[c.x,c.z,c.y] , size=[c.length,c.width,c.height])
        pyrosim.End()
    
    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[c.length,c.width,c.height])
        pyrosim.Send_Joint( name = "Torso_BackLeg", parent = "Torso", child = "BackLeg", type = "revolute", position = [1,0,1])
        pyrosim.Send_Cube(name="BackLeg", pos=[-.5,0,-.5] , size=[c.length,c.width,c.height])
        pyrosim.Send_Joint( name = "Torso_FrontLeg", parent = "Torso", child = "FrontLeg", type = "revolute", position = [2,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[.5,0,-.5] , size=[c.length,c.width,c.height])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain"+str(self.myID)+".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
        for currentRow in range(3):
            for currentColumn in range(2):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+3 , weight = self.weights[currentRow][currentColumn])
    
        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0,2)
        randomCol = random.randint(0,1)
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
        #print(self.fitness)
        
        
        
        