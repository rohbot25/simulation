from solution import SOLUTION
import constants as c
import copy
import numpy as np
import os

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm data/fitness*.txt")
        os.system("rm data/x*.txt")
        self.parents = {}
        self.nextAvailableID = 0
        self.fitnessMatrix = np.zeros([c.numberOfGenerations,c.populationSize])
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self,i):
        self.Evaluate(self.parents,0,i)
        for currentGeneration in range(c.numberOfGenerations-1):
            self.Evolve_For_One_Generation(currentGeneration+1,i)

    def Evolve_For_One_Generation(self,gen,i):
        self.Spawn()

        self.Mutate()

        self.Evaluate(self.children,gen,i)

        self.Print()
        
        self.Select()

    def Spawn(self):
        self.children = {}
        for key in self.parents:
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID+=1
    def Mutate(self):
        for key in self.children:
            self.children[key].Mutate()
        
    def Select(self):
        for keys in self.parents:
            if(self.parents[keys].fitness < self.children[keys].fitness):
                self.parents[keys] = self.children[keys]

    def Print(self):
        for keys in self.parents:
            print("\n\nParent: "+str(self.parents[keys].fitness),"Child: "+str(self.children[keys].fitness)+"\n\n")

    def Show_Best(self):
        parent = self.parents[0]
        for keys in self.parents:
            if self.parents[keys].fitness > parent.fitness:
                parent = self.parents[keys]
        parent.Start_Simulation("GUI",1)

    def Evaluate(self,solutions,gen,i):
        pop = 0
        for key in solutions:
            solutions[key].Start_Simulation("DIRECT",i)
        for key in solutions:
            solutions[key].Wait_For_Simulation_To_End()
        for key in solutions:
            self.fitnessMatrix[gen][pop] = solutions[key].x
            pop += 1
            
