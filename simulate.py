import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import pybullet_data
import time
import numpy
import random

#set up physics sim
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,c.gravity)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)

#fill sensor arrays
backLegSensorValues = numpy.zeros(c.iterations)
frontLegSensorValues = numpy.zeros(c.iterations)
frontTargetAngles = numpy.zeros(c.iterations)
backTargetAngles = numpy.zeros(c.iterations)

#get values 0 to 2pi
x = numpy.linspace(0,c.maxValues,c.iterations)
#fill angles with sin values
for a in range(c.iterations):
    frontTargetAngles[a] = c.frontAmplitude * numpy.sin(c.frontFrequency * x[a] + c.frontPhaseOffset)
    backTargetAngles[a] = c.backAmplitude * numpy.sin(c.backFrequency * x[a] + c.backPhaseOffset)
#numpy.save("data/sin.npy",targetAngles)
#run the simulation
for i in range(c.iterations):
    time.sleep(c.sleepy)
    p.stepSimulation()
    #sensors
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    #backleg motor
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = b'Torso_BackLeg',
        controlMode = p.POSITION_CONTROL,
        targetPosition = backTargetAngles[i],
        maxForce = c.force)
    #frontleg motor
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = b'Torso_FrontLeg',
        controlMode = p.POSITION_CONTROL,
        targetPosition = frontTargetAngles[i],
        maxForce = c.force)

p.disconnect()
numpy.save("data/backLeg.npy",backLegSensorValues)
numpy.save("data/frontLeg.npy",frontLegSensorValues)
