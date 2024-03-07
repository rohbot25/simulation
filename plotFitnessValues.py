import constants as c
import numpy as np
import matplotlib.pyplot

raw = []
means = []
titles = ["x * z","f","n","h","f+n","n+h","f+h","f+n+h"]
for i in range(c.ab):
    with open('fitness'+str(i)+'.npy','rb') as f:
        raw.append(np.load(f))



for run in raw:
    temp = []
    for array in run:
        temp.append(np.mean(array))

    means.append(temp)



for i in range(c.ab):
    matplotlib.pyplot.plot(means[i], label = titles[i],linewidth = 1)

matplotlib.pyplot.title("Changing the fitness Function")
matplotlib.pyplot.xlabel("# of Generations")
matplotlib.pyplot.ylabel("Distance in X-Direction")

matplotlib.pyplot.legend()
matplotlib.pyplot.show()
    

