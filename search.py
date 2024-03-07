import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import numpy as np
import constants as c

phc = PARALLEL_HILL_CLIMBER()
for i in range(c.ab):
    phc.Evolve(i)
    #phc.Show_Best()

    with open('fitness'+str(i)+'.npy','wb') as f:
        np.save(f,phc.fitnessMatrix)
