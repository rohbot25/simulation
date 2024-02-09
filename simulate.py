import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import pybullet_data
import time
import numpy
import random
from simulation import SIMULATION
import sys

directOrGUI = sys.argv[1]
simulation = SIMULATION(directOrGUI)

simulation.Run()

simulation.Get_Fitness()



