import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import matplotlib.pyplot as plt
import pickle

with open('phc.pkl','rb') as f:
    phc = pickle.load(f)
phc.Evolve()
f = open('phc.pkl','wb')
pickle.dump(phc,f)
f.close()
#phc.Show_Best()


xz = []
f = open("data/xzFinal.txt",'w')

for i in range(len(phc.best)):
    xz.append(float(phc.best[i][0]))
    xz.append(float(phc.best[i][1]))
    xz.append(float(phc.best[i][2]))
    xz.append(float(phc.best[i][3]))
    
f.write(str(xz))
f.close()
