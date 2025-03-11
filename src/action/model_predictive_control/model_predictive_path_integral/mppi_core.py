import numpy as np
import math

class mppi_core:
    def __init__(self):
        self.numPaths = 500 # number of paths
        self.numTimeSteps = 50 #number of time steps
        self.TIME_STEP = 0.05
        self.controlSequence = np.zeros((2,self.numTimeSteps))
        self.PrevControlSequence = np.zeros((2,self.numTimeSteps))
        self.controlPaths = np.zeros((2,self.numTimeSteps,self.numPaths))
        self.corresponding_dynamics = np.zeros((4,self.numTimeSteps,self.numPaths))

        self.initialRun = True
        self.x0 = 0
        self.y0 = 0
        self.theta0 = 0
        self.v0 = 0
        self.std_dev_accel = 0
        self.std_dev_theta = 0
    def generate_controls(self): #random control sequences
        
        if self.initialRun:
            self.PrevControlSequence[1:] = self.theta0
        
        ThetaNoise = np.random.normal(self.theta0, self.std_dev_theta, size=(self.numTimeSteps, self.numPaths))
        AccelNoise = np.random.normal(0, self.std_dev_accel, size=(self.numTimeSteps, self.numPaths))

        self.controlPaths[0,:,:] = self.PrevControlSequence[0,:np.newaxis] + ThetaNoise
        self.controlPaths[1,:,:] = self.PrevControlSequence[1,:np.newaxis] + AccelNoise

    def simulate_trajectory(self,accel, heading):
	    X[0] =self.x0
	    Y[0] = self.y0
	    V[0] = self.v0
	    T[0] = self.theta0
	    for i in range(1,self.numTimeSteps):
            X[i] = X[i-1] + V[i-1]*math.cos(T[i-1])*self.TIME_STEP
            Y[i] = Y[i-1] + V[i-1]*math.sin(T[i-1])*self.TIME_STEP
            V[i] = V[i-1]+ accel[i-1]*time_step
            T[i] = T[i-1] + (V[i-1]/L) * math.tan(heading[i-1]) *self.TIME_STEP
	    return X,Y,V,T 


    def compute_costs(self):
        #at each time step 
        pass


    def integrate_solution(self):
        pass