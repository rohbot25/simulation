import numpy
import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import pybullet_data
class MOTOR:
    def __init__(self,jointName):
        self.jointName = jointName
        self.motorValues = numpy.zeros(c.iterations)
        self.Prepare_to_Act()
        
    def Prepare_to_Act(self):
        self.amplitude = c.amplitude
        if "Front" in str(self.jointName):
            self.frequency = c.frequency/2
        else:
            self.frequency = c.frequency
        self.offset = c.offset
        x = numpy.linspace(0,c.maxValues,c.iterations)
        for a in range(c.iterations):
            self.motorValues[a] = self.amplitude * numpy.sin(self.frequency * x[a] + self.offset)
            
    def Set_Value(self,t,robotId):
        pass
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = robotId,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = self.motorValues[t],
            maxForce = c.force)
    def Save_Values(self):
        numpy.save("data/"+self.jointName+".npy",self.motorValues)
        
