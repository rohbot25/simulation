import pybullet as p

physicsClient = p.connect(p.GUI)
p.stepSimulation()
p.disconnect()
