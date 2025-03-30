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
        self.controlPaths = np.zeros((2,self.numTimeSteps,self.numSamples))
        self.corresponding_dynamics = np.zeros((4,self.numTimeSteps,self.numSamples))

        self.initialRun = True
        self.L=0
        self.x0 = 0
        self.y0 = 0
        self.theta0 = 0
        self.v0 = 0
        self.std_dev_accel = 0
        self.std_dev_theta = 0
        self.lamda_temp =0
        self.waypoints = []
        self.costs = np.zeros(self.numSamples)
        self.weights = np.zeros(self.numSamples)
        self.Q_state = np.diag(0,0,0)
        self.Q_control = np.diag(0,0,0)
        self.Q_smoothness = np.diag(0,0,0)
        self.Q_terminal = np.diag(0,0,0)
    def generate_control_samples(self): # random control sequences
        
        if self.initialRun: #use previous control sequence
            self.PrevControlSequence[1:] = self.theta0
            self.initialRun = False

        ThetaNoise = np.zeros((self.numSamples, self.numTimeSteps))
        AccelNoise = np.zeros((self.numSamples, self.numTimeSteps))
        
        for i in range(0, self.numSamples):
            ThetaNoise[i, :] = np.random.normal(self.theta0, self.std_dev_theta, size=(self.numTimeSteps, self.numPaths))
            AccelNoise[i, :] = np.random.normal(0, self.std_dev_accel, size=(self.numTimeSteps, self.numPaths))

            self.controlPaths[0, i, :] = self.PrevControlSequence[0,: self.numTimeSteps] + ThetaNoise
            self.controlPaths[1,i , :] = self.PrevControlSequence[1,: self.numTimeSteps] + AccelNoise

    def simulate_trajectory(self,accel, heading,X,Y,V,T): #write x,y,V,T arrays for simulataed trajectories
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
        costs = []
        #must find closest way point in time to use as reference for state error
        for i in range(0,self.numSamples): #for every sample
            self.simulate_trajectory(self.controlPaths[0,i,:],self.controlPaths[1,i,:], X,Y,V,T)
            cost = 0

            for j in range(1,self.numTimeSteps): #for every time step
              
                X_ref,Y_ref,T_ref = self.find_closest_waypoint(j,self.Waypoints) #get reference point, add V_ref?
                
                #get state error
                state_actual = np.array([X[j],Y[j],T[j]])
                state_ref = np.array([X_ref,Y_ref,T_ref])
                state_error = state_ref-state_actual
                
                #get state cost
                state_cost = state_error.T @ self.Q_state @state_error

                #get control effort
                U_current = np.array(self.controlPaths[:,i,j])#get both accel and heading for one timestep of one path
                U_prev = np.array(self.controlPaths[:,i,j-1])
                U_effort = U_current - U_prev
                #inputs (Control and smoothness) Cost
                smoothness_cost = U_effort.T @ self.Q_control @ U_effort
                control_cost = U_current @ self.Q_smoothness @ U_current

                #obstacle cost not yet
            

            #terminal cost
            terminal_cost = state_error.T @ self.Q_terminal @ state_error
            #total cost
            cost = state_cost + smoothness_cost + control_cost + terminal_cost
            costs[i] = cost
        self.costs = np.array(costs)


             
    def find_closest_waypoint(self,time_step):
        closestPoint = self.waypoints[0]
        minTimeDiff = time_step - self.waypoints[0][0] 
        for i in self.waypoints:
            if i[0] < minTimeDiff:
                closestPoint = i

        return  closestPoint[1], closestPoint[2], closestPoint[0]



    def compute_exponential_weights(self): #weight of each sample
        min_cost = np.min(self.costs)
        self.weights = np.exp(-(self.costs - min_cost)/ self.lamda_temp) # weight is e^(-J(k)-min(J)/lambda)
        
        self.weights /= np.sum(self.weights) # normalize weights so they all add up to 1


    def integrate_solution(self):
        weighted_paths = self.controlPaths * self.weights[np.newaxis, np.newaxis, :]
        self.outControlSequence = np.sum(weighted_paths,axis=2) #sum over the numSamples Axis