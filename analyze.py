import matplotlib.pyplot
import numpy

backLegSensorValues = numpy.load("data/backLeg.npy")
frontLegSensorValues = numpy.load("data/frontLeg.npy")

matplotlib.pyplot.plot(backLegSensorValues, label = 'Back Leg',linewidth = 3)
matplotlib.pyplot.plot(frontLegSensorValues, label = 'Front Leg')
matplotlib.pyplot.legend()
matplotlib.pyplot.show()

