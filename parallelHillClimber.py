from solution import SOLUTION
import constants as c
import copy
import os

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm data/fitness*.txt")
        self.parents = {}
        self.nextAvailableID = 0
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()

        self.Mutate()

        self.Evaluate(self.children)

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
        parent.Start_Simulation("GUI")

    def Evaluate(self,solutions):
        for key in solutions:
            solutions[key].Start_Simulation("DIRECT")
        for key in solutions:
            solutions[key].Wait_For_Simulation_To_End()
