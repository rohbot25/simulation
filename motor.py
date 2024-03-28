import numpy
import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import pybullet_data
class MOTOR:
    def __init__(self,jointName):
        self.jointName = jointName
        self.motorValues = numpy.zeros(c.iterations)
            
    def Set_Value(self,desiredAngle,robotId):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = robotId,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = desiredAngle,
            maxForce = c.force)
        
