import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import matplotlib.pyplot as plt
import pickle

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
f = open('phc.pkl','wb')
pickle.dump(phc,f)
f.close()
phc.Show_Best()


xz = []
f = open("data/xzFinal.txt",'w')

for i in range(len(phc.best)):
    xz.append(float(phc.best[i][0]))
    xz.append(float(phc.best[i][1]))
    xz.append(float(phc.best[i][2]))
    xz.append(float(phc.best[i][3]))
f.write(str(xz))
f.close()


    


