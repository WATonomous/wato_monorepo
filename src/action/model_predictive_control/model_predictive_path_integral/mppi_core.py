import numpy as np


class mppi_core:
    def __init__(self):
        self.numPaths = 500 # number of paths
        self.numTimeSteps = 50 #number of time steps
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
        pass
    def generate_controls(self): #random control sequences
        
        if self.initialRun:
            self.PrevControlSequence[1:] = self.theta0
        ThetaNoise = np.random.normal(self.theta0, self.std_dev_theta, size=(self.numTimeSteps, self.numPaths))
        AccelNoise = np.random.normal(0, self.std_dev_accel, size=(self.numTimeSteps, self.numPaths))

        self.controlPaths[0,:,:] = self.PrevControlSequence[0,:np.newaxis] + ThetaNoise
        self.controlPaths[1,:,:] = self.PrevControlSequence[1,:np.newaxis] + AccelNoise

    def simulate_trajectories(self):
        
        for i in range(1,self.numPaths):#for every column in this array calculate the x,y,v,theta
            #corresponding_dynamics[:,i][0]= corresponding_dynamics[:,i-1][0] + 
        #need to update the x,y,theta,velocity

    def compute_costs(self):
        pass


    def integrate_solution(self):
        pass