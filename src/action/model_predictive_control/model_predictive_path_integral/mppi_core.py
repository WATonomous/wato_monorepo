import numpy as np
import math

class mppi_core:
    def __init__(self): 
        self.numSamples = 500 # number of samples/path to generate at each timesetep
        self.numTimeSteps = 50 # number of time steps
        self.TIME_STEP = 0.05 # seconds per time step

        self.prevControlSequence = np.zeros((2,self.numTimeSteps)) # input
        self.outControlSequence = np.zeros((2,self.numTimeSteps)) # output
        
        # 
        self.controlPaths = np.zeros((2,self.numTimeSteps,self.numPaths))
        self.corresponding_dynamics = np.zeros((4,self.numTimeSteps,self.numPaths))

        self.initialRun = True
        self.L=0
        self.x0 = 0
        self.y0 = 0
        self.theta0 = 0
        self.v0 = 0
        self.std_dev_accel = 0
        self.std_dev_theta = 0
        pass
    
    def generate_control_samples(self): # random control sequences
        
        if self.initialRun:
            self.PrevControlSequence[1:] = self.theta0
            self.initialRun = False

        ThetaNoise = np.zeros((self.numSamples, self.numTimeSteps))
        AccelNoise = np.zeros((self.numSamples, self.numTimeSteps))
        
        for i in range(0, self.numSamples):
            ThetaNoise[i, :] = np.random.normal(self.theta0, self.std_dev_theta, size=(self.numTimeSteps, self.numPaths))
            AccelNoise[i, :] = np.random.normal(0, self.std_dev_accel, size=(self.numTimeSteps, self.numPaths))

            self.controlPaths[0, i, :] = self.PrevControlSequence[0,: self.numTimeSteps] + ThetaNoise
            self.controlPaths[1,i , :] = self.PrevControlSequence[1,: self.numTimeSteps] + AccelNoise

    def simulate_trajectory(self,accel, heading,X,Y,V,T):
        X[0] =self.x0
        Y[0] = self.y0
        V[0] = self.v0
        T[0] = self.theta0
        for i in range(1,self.numTimeSteps):
            X[i] = X[i-1] + V[i-1]*math.cos(T[i-1])*self.TIME_STEP
            Y[i] = Y[i-1] + V[i-1]*math.sin(T[i-1])*self.TIME_STEP
            V[i] = V[i-1]+ accel[i-1]*self.TIME_STEP
            T[i] = T[i-1] + (V[i-1]/self.L) * math.tan(heading[i-1]) *self.TIME_STEP
        #return X,Y,V,T 


    def compute_costs(self):
        #at each time step
        X = np.zeros(self.numTimeSteps)
        Y = np.zeros(self.numTimeSteps)
        V = np.zeros(self.numTimeSteps)
        T = np.zeros(self.numTimeSteps)
        cost = 0
        for i in range(0,self.numSamples):
            self.simulate_trajectory(self.controlPaths[0,i,:],self.controlPaths[1,i,:], X,Y,V,T)
            for i in range(1,self.numTimeSteps):
            #state error
                cost += (X[i] - waypointsX)**2
                cost += (Y[i] - waypointsY)**2
            #Control Cost
                cost += (self.controlPaths[0,i,:] - self.controlPaths[0,i-1,:])**2 #for acceleration
                cost += (self.controlPaths[1,i,:] - self.controlPaths[1,i-1,:])**2 #for heading
            #terminal cost

            #obstacle cost
            


    def integrate_solution(self):
        ##add exponential weighting where the lower costs are given more weight
        pass