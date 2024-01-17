import pybullet as p
import time

physicsClient = p.connect(p.GUI)
p.loadSDF("box.sdf")
for i in range(1000):
    time.sleep(1/60)
    print(i)
    p.stepSimulation()

p.disconnect()
