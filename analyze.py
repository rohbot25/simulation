import matplotlib.pyplot
import numpy

backLegSensorValues = numpy.load("data/backLeg.npy")
frontLegSensorValues = numpy.load("data/frontLeg.npy")
#targetAngles = numpy.load("data/sin.npy")
matplotlib.pyplot.plot(backLegSensorValues, label = 'Back Leg',linewidth = 3)
matplotlib.pyplot.plot(frontLegSensorValues, label = 'Front Leg')
#matplotlib.pyplot.plot(targetAngles, label = 'TargetAngles')
matplotlib.pyplot.legend()
matplotlib.pyplot.show()

