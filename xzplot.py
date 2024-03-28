import matplotlib.pyplot as plt
import constants as c
import numpy as np


raw = []
x = []
z = []
a = []
j = []
xz = open("data/aFinal.txt",'r')

raw = xz.read().replace("[","").replace("]","").split(',')


for i in range(len(raw)):
    if(i%4 == 0):
        x.append(float(raw[i]))
    elif(i%4 == 1):
        z.append(float(raw[i]))
    elif(i%4 == 2):
        a.append(float(raw[i]))
    else:
        j.append(float(raw[i]))
        

xz.close()
        
fig , ax1 = plt.subplots()
ax2 = ax1.twinx()
ax1.plot(x, label = 'x',color = 'blue',  linewidth = 1)
ax1.plot(z, label = 'z',color = 'blue', linewidth = 2)
ax2.plot(a, label = 'a',color = 'red', linewidth = 1)
ax2.plot(j, label = 'j',color = 'red', linewidth = 2)


ax1.set_xlabel("# of Generations")
ax1.set_ylabel("Body Length's", color = 'blue')

ax2.set_ylabel("d/t^2",color = 'red')

fig.suptitle("Biped Fitness Function (X * Z / (1+j))")

ax1.legend()
ax2.legend()

plt.show()
