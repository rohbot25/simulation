import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
import time
import numpy
import random

iterations = 500
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = numpy.zeros(iterations)
frontLegSensorValues = numpy.zeros(iterations)
for i in range(iterations):
    time.sleep(1/60)
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = b'Torso_BackLeg',
        controlMode = p.POSITION_CONTROL,
        targetPosition = -numpy.pi/2 + random.random() * numpy.pi,
        maxForce =100)
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = b'Torso_FrontLeg',
        controlMode = p.POSITION_CONTROL,
        targetPosition = -numpy.pi/2 + random.random() * numpy.pi,
        maxForce =100)

p.disconnect()
numpy.save("data/backLeg.npy",backLegSensorValues)
numpy.save("data/frontLeg.npy",frontLegSensorValues)
