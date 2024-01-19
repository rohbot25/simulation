import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1

x = 0
y = 0.5
z = 0
pyrosim.Start_SDF("boxes.sdf")
for i in range(10):
    pyrosim.Send_Cube(name="Box", pos=[x,z,y] , size=[length,width,height])
    length*=.9
    width*=.9
    previousHeight = .5 * height
    height*=.9
    y = y + previousHeight + .5 * height


pyrosim.End()
