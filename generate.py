import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1

x = 0
y = 0.5
z = 0
pyrosim.Start_SDF("boxes.sdf")
for i in range(5):
    x = 0
    for j in range(5):
        length = 1
        width = 1
        height = 1
        y = 0.5
        x += 1
        for k in range(10):
            pyrosim.Send_Cube(name="Box", pos=[x,z,y] , size=[length,width,height])
            length*=.9
            width*=.9
            previousHeight = .5 * height
            height*=.9
            y = y + previousHeight + .5 * height
    z += 1


pyrosim.End()
