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
        self.controlPaths = np.zeros((2,self.numTimeSteps,self.numSamples)) #change to controlSamples
        self.corresponding_dynamics = np.zeros((4,self.numTimeSteps,self.numSamples))

        self.initialRun = True
        self.L=2.875  # Wheelbase of the vehicle. Source : https://www.tesla.com/ownersmanual/model3/en_us/GUID-56562137-FC31-4110-A13C-9A9FC6657BF0.html
        self.x0 = 0
        self.y0 = 0
        self.theta0 = 0
        self.v0 = 0
        self.std_dev_accel = 0
        self.std_dev_steer_angle = 0
        self.steer_constraints=[-1,1]
        self.throttle_constraints=[-1,1]


        self.lamda_temp =0
        self.waypoints = []
        self.costs = np.zeros(self.numSamples)
        self.weights = np.zeros(self.numSamples)
        self.Q_state = np.diag([0,0,0])
        self.Q_control = np.diag([0,0,0])
        self.Q_smoothness = np.diag([0,0,0])
        self.Q_terminal = np.diag([0,0,0])
    def generate_control_samples(self): # random control sequences
        
        #if self.initialRun: #use previous control sequence
        #    self.prevControlSequence[1,:] = 0 
        #   self.initialRun = False
        
        #what offset do you put on the indexes of the control sequences
        for i in range(0, self.numSamples):
            Steer_angle_Noise = np.random.normal(0, self.std_dev_steer_angle, size=(self.numTimeSteps,)) #add comma to classify it as a tuple
            AccelNoise = np.random.normal(0, self.std_dev_accel, size=(self.numTimeSteps,))
            self.controlPaths[0, :, i] = self.prevControlSequence[0,:] + Steer_angle_Noise
            self.controlPaths[1,:, i] = self.prevControlSequence[1,:] + AccelNoise
        self.controlPaths[0,:,:] = np.clip(self.controlPaths[0,:,:],self.steer_constraints[0],self.steer_constraints[1])
        self.controlPaths[1,:,:] = np.clip(self.controlPaths[1,:,:],self.throttle_constraints[0],self.throttle_constraints[1])

    def simulate_trajectory(self,accel, steer_angle): #write x,y,V,T arrays for simulataed trajectories
        X = np.zeros(self.numTimeSteps)
        Y = np.zeros(self.numTimeSteps)
        V = np.zeros(self.numTimeSteps)
        T = np.zeros(self.numTimeSteps)   
        X[0] =self.x0
        Y[0] = self.y0
        V[0] = self.v0
        T[0] = self.theta0
        for i in range(1,self.numTimeSteps):
            X[i] = X[i-1] + V[i-1]*math.cos(T[i-1])*self.TIME_STEP
            Y[i] = Y[i-1] + V[i-1]*math.sin(T[i-1])*self.TIME_STEP
            V[i] = V[i-1]+ accel[i-1]*self.TIME_STEP
            T[i] = T[i-1] + (V[i-1]/self.L) * math.tan(steer_angle[i-1]) *self.TIME_STEP
        return X,Y,V,T 


    def compute_costs(self):
        #at each time step
        X = np.zeros(self.numTimeSteps)
        Y = np.zeros(self.numTimeSteps)
        V = np.zeros(self.numTimeSteps)
        T = np.zeros(self.numTimeSteps)
        costs = []
        #must find closest way point in time to use as reference for state error
        for i in range(0,self.numSamples): #for every sample
            X,Y,V,T = self.simulate_trajectory(self.controlPaths[0,i,:],self.controlPaths[1,i,:])
            cost = 0
            state_cost=0
            control_cost=0
            smoothness_cost=0
            terminal_cost =0
            for j in range(1,self.numTimeSteps): #for every time step  
                X_ref,Y_ref,T_ref = self.find_closest_waypoint(j,self.Waypoints) #get reference point, add V_ref? remove T_ref?
                
                #get state error
                state_actual = np.array([X[j],Y[j],T[j]])
                state_ref = np.array([X_ref,Y_ref,T_ref])
                state_error = state_ref-state_actual
                
                #get state cost
                state_cost += state_error.T @ self.Q_state @state_error

                #get control effort
                U_current = np.array(self.controlPaths[:,j,i])#get both accel and heading for one timestep of one path
                U_prev = np.array(self.controlPaths[:,j-1,i])
                U_effort = U_current - U_prev
                #inputs (Control and smoothness) Cost
                smoothness_cost += U_effort.T @ self.Q_control @ U_effort
                control_cost += U_current.T @ self.Q_smoothness @ U_current

                #obstacle cost not yet
            #terminal cost
            terminal_cost += state_error.T @ self.Q_terminal @ state_error
            #total cost
            cost = state_cost + smoothness_cost + control_cost + terminal_cost
            costs.append(cost)
        self.costs = np.array(costs)


    def find_closest_waypoint(self,num_timesteps_in): #x y,time 
        #start with closest point, eventually interpolate it!!
        #waypoint structure: x,y,theta,time
        closestPoint = self.waypoints[0]
        minTimeDiff = abs(num_timesteps_in*self.TIME_STEP - self.waypoints[0][3])
        for wp in self.waypoints:
            time_diff = abs(num_timesteps_in - wp[3])
            if time_diff < minTimeDiff:
                minTimeDiff=time_diff
                closestPoint = wp
        return  closestPoint[0], closestPoint[1],closestPoint[2] #x,y,theta


    def compute_exponential_weights(self): #weight of each sample
        min_cost = np.min(self.costs)
        self.weights = np.exp(-(self.costs - min_cost)/ self.lamda_temp) # weight is e^(-J(k)-min(J)/lambda)
        self.weights /= np.sum(self.weights) # normalize weights so they all add up to 1


    def integrate_solution(self):
        weighted_paths = self.controlPaths * self.weights[np.newaxis, np.newaxis, :]
        self.outControlSequence =   np.sum(weighted_paths,axis=2) #sum over the numSamples Axis
        return self.outControlSequence[0,0], self.outControlSequence[1,0]#return first control input only


    def compute_controls(self):
        self.prevControlSequenceControlSequence = self.outControlSequence
        self.generate_control_samples()
        self.compute_costs()
        self.compute_exponential_weights()
        steerangle,accel = self.integrate_solution()
        return steerangle,accel
    