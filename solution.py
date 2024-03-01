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
        self.weights = self.weights * 0

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        #pyrosim.Send_Cube(name="Box", pos=[c.x,c.y,c.z] , size=[c.length,c.width,c.height])
        pyrosim.End()
    
    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        #pyrosim.Send_Cube(name="Head", pos=[] , size = [])
        #pyrosim.Send_Cube(name="Neck", pos=[] , size = [])
##        pyrosim.Send_Cube(name="Shoulder", pos=[] , size = [])
##        pyrosim.Send_Joint( name = "Shoulder_LeftUpperArm", parent = "Shoulder", child = "LeftUpperArm", type = "revolute", position = [0,-0.5,1],jointAxis = "1 0 0")
##        pyrosim.Send_Joint( name = "Shoulder_RightUpperArm", parent = "Shoulder", child = "RightUpperArm", type = "revolute", position = [0,-0.5,1],jointAxis = "1 0 0")
##        pyrosim.Send_Cube(name="LeftUpperArm", pos=[] , size = [])
##        pyrosim.Send_Joint( name = "LeftUpperArm_LeftForearm", parent = "LeftUpperArm", child = "LeftForearm", type = "revolute", position = [0,-0.5,1],jointAxis = "1 0 0")
##        pyrosim.Send_Cube(name="LeftForearm", pos=[] , size = [])
##        pyrosim.Send_Cube(name="RightUpperArm", pos=[] , size = [])
##        pyrosim.Send_Joint( name = "RightUpperArm_RightForearm", parent = "RightUpperArm", child = "RightForearm", type = "revolute", position = [0,-0.5,1],jointAxis = "1 0 0")
##        pyrosim.Send_Cube(name="RightForearm", pos=[] , size = [])


        pyrosim.Send_Cube(name="RightFoot", pos=[0,0,0.1] , size = [.25,.2,.2])
        pyrosim.Send_Joint( name = "RightFoot_RightTibia", parent = "RightFoot", child = "RightTibia", type = "revolute", position = [0,0,0.1],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="RightTibia", pos=[0,0,.5*c.size] , size = [.2,.2,1*c.size])
        pyrosim.Send_Joint( name = "RightTibia_RightFemur", parent = "RightTibia", child = "RightFemur", type = "revolute", position = [0,0,1*c.size],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="RightFemur", pos=[0,0,.5*c.size] , size = [.2,.2,1*c.size])
        pyrosim.Send_Joint( name = "RightFemur_Hip", parent = "RightFemur", child = "Hip", type = "revolute", position = [0,0,1*c.size],jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="Hip", pos=[0,-0.25,0] , size = [.2,.5,.2])
        pyrosim.Send_Joint(name = "Hip_LeftFemur", parent = "Hip", child = "LeftFemur", type = "revolute", position = [0,-0.5,0],jointAxis = "0 -1 0")
        pyrosim.Send_Cube(name="LeftFemur", pos=[0,0,-0.5*c.size] , size = [.2,.2,1*c.size])
        pyrosim.Send_Joint( name = "LeftFemur_LeftTibia", parent = "LeftFemur", child = "LeftTibia", type = "revolute", position = [0,0,-1*c.size],jointAxis = "0 -1 0")
        pyrosim.Send_Cube(name="LeftTibia", pos=[0,0,-0.5*c.size] , size = [.2,.2,1*c.size])
        pyrosim.Send_Joint( name = "LeftTibia_LeftFoot", parent = "LeftTibia", child = "LeftFoot", type = "revolute", position = [0,0,-1*c.size+0.1],jointAxis = "0 -1 0")
        pyrosim.Send_Cube(name="LeftFoot", pos=[0,0,-0.1] , size = [.25,.2,.2])
        
        #pyrosim.Send_Cube(name="Torso", pos=[] , size = [])
        #pyrosim.Send_Joint( name = "Torso_Hip", parent = "Torso", child = "Hip", type = "revolute", position = [0,-0.5,1],jointAxis = "1 0 0")
        
        
        
        
        
        pyrosim.End()

    def Create_Brain(self):
        pass
        pyrosim.Start_NeuralNetwork("brain"+str(self.myID)+".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "RightFoot")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "RightTibia")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "RightFemur")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "Hip")
        pyrosim.Send_Sensor_Neuron(name = 4, linkName = "LeftFemur")
        pyrosim.Send_Sensor_Neuron(name = 5, linkName = "LeftTibia")
        pyrosim.Send_Sensor_Neuron(name = 6, linkName = "LeftFoot")
        pyrosim.Send_Motor_Neuron(name = 7, jointName = "RightFoot_RightTibia")
        pyrosim.Send_Motor_Neuron(name = 8, jointName = "RightTibia_RightFemur")
        pyrosim.Send_Motor_Neuron(name = 9, jointName = "RightFemur_Hip")
        pyrosim.Send_Motor_Neuron(name = 10, jointName = "Hip_LeftFemur")
        pyrosim.Send_Motor_Neuron(name = 11, jointName = "LeftFemur_LeftTibia")
        pyrosim.Send_Motor_Neuron(name = 12, jointName = "LeftTibia_LeftFoot")
        
        
        for currentRow in range(c.numSensorNeurons):
            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+c.numSensorNeurons , weight = self.weights[currentRow][currentColumn])

        pyrosim.End()
        
    def Mutate(self):
        randomRow = random.randint(0,c.numSensorNeurons-1)
        randomCol = random.randint(0,c.numMotorNeurons-1)
        self.weights[randomRow][randomCol] = random.random()


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
        
        
        
        
