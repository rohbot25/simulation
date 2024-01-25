import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
import time
import numpy
import random

#variables
iterations = 1000
force = 100
frontTargetAngles = numpy.zeros(iterations)
backTargetAngles = numpy.zeros(iterations)
backAmplitude = numpy.pi/4
backFrequency = 20
backPhaseOffset = 0
frontAmplitude = numpy.pi/3
frontFrequency = 10
frontPhaseOffset = 0

#set up physics sim
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)
#fill sensor arrays
backLegSensorValues = numpy.zeros(iterations)
frontLegSensorValues = numpy.zeros(iterations)

#get values 0 to 2pi
x = numpy.linspace(0,2*numpy.pi,iterations)
#fill angles with sin values
for a in range(iterations):
    frontTargetAngles[a] = frontAmplitude * numpy.sin(frontFrequency * x[a] + frontPhaseOffset)
    backTargetAngles[a] = backAmplitude * numpy.sin(backFrequency * x[a] + backPhaseOffset)
#numpy.save("data/sin.npy",targetAngles)
#run the simulation
for i in range(iterations):
    time.sleep(1/240)
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
        maxForce =force)
    #frontleg motor
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = b'Torso_FrontLeg',
        controlMode = p.POSITION_CONTROL,
        targetPosition = frontTargetAngles[i],
        maxForce =force)

p.disconnect()
numpy.save("data/backLeg.npy",backLegSensorValues)
numpy.save("data/frontLeg.npy",frontLegSensorValues)
