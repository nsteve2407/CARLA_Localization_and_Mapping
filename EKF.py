import numpy as np



class EKF():
    def __init__(self):
        self.x = np.array[0.0,0.0,0.0] # 3D state vector [x,y,theta]
        self.P = np.zeros((3,3),dtype=np.float)
        self.Q = np.zeros((3,3),dtype=np.float)
        self.R_lidar = np.zeros((3,3),dtype=np.float)


    def predict(delta_t): 
        #Inputs
        # takes input as the delta time betweem previous state estiate and current estimate
        #Outputs
        # Updates state(self.x)

        # To do
        # Implement constant velocity motion model to update state(self.x)
        # Define A matrix and use state transftion function


        return 0

    
    def measurement_update_lidar(lidar_measurement):
        # Input:
        # Lidar measurement is the lidar position estimate obtained from Scan Matching
        # Lidar measurement is of the form [x,y,theta]

        # Output
        # Updates the state variable (self.x)

        # To do :
        # # Write code for measurement update 

        return 0

    def measurement_update_gps(gps_measurement):
        # Input:
        # GPS measurement is the noisy measurement of the vehicle's position  obtained from the simulator
        # GPS measurement is of the form [x,y,theta]

        # Output
        # Updates the state variable (self.x)

        # To do :
        # # Write code for measurement update 

        return 0


