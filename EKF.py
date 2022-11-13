import numpy as np



class EKF():
    def __init__(self):
        self.x = np.array[0.0,0.0,0.0,0.0,0.0,0.0] # 6D state vector [x,y,theta,x',y',theta']
        self.P = np.eye((6,6),dtype=np.float)
        self.Q = np.eye((6,6),dtype=np.float)
        self.R_lidar = np.eye((3,3),dtype=np.float)


    def predict(self,delta_t): 

        # Inputs

        x_t = 0
        x_t_dot = 0
        y_t = 0
        y_t_dot = 0
        theta_t = 0
        theta_t_dot = 0
        P_t = 0

        x_t_plus_1 = x_t + x_t_dot @ delta_t
        y_t_plus_1 = y_t + y_t_dot @ delta_t
        theta_t_plus_1 = theta_t + theta_t_dot @ delta_t

        x_t_plus_1_dot = x_t_dot 
        y_t_plus_1_dot = y_t_dot
        theta_t_plus_1_dot = theta_t_dot

        X_t = [x_t, y_t, theta_t, x_t_dot, y_t_dot, theta_t_dot]

        X_t_plus_1 = [x_t_plus_1, y_t_plus_1, theta_t_plus_1, x_t_plus_1_dot, y_t_plus_1_dot, theta_t_plus_1_dot] #not sure

        # State transition matrix A at timestep t-1
        
        A_t = np.eye(6, dtype = float)

        # Covariance
        
        P_t_plus_1 = A_t @ P_t @ A_t.T + self.Q

        # takes input as the delta time betweem previous state estiate and current estimate
        X_t_plus_1 = A_t @ X_t.T    #not sure
       
        self.x = A_t @ X_t.T        
    
        # Output
        # Updates state(self.x)
        # To do
        # Implement constant velocity motion model to update state(self.x)
        # Define A matrix and use state transftion function
        return 0

    
    def measurement_update_lidar(self,lidar_measurement):
        # Input:
        # Lidar measurement is the lidar position estimate obtained from Scan Matching
        # Lidar measurement is of the form [x,y,theta]

        #Your code here

        self.H = np.array[1.0,1.0,1.0,1.0,1.0,1.0] #6*6

        y = lidar_measurement - np.dot(self.H, self.x) #line 27 for z lidar mesa

        S_t = H_t @ P_t @ H_t.T + R_lidar # mesurement residual covariance r lidar

        K_t = P_t @ H_t.T @ np.linalg.pinv(S_t) # Kalman gain

        self.x = self.x + np.dot(K, y)

        I = np.eye(6,6) #np.ones

        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
                (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R_lidar), K.T)

        # Output
        # Updates the state variable (self.x)

        # To do :
        # # Write code for measurement update 

        return 0

    def measurement_update_gps(self,gps_measurement):
        # Input:
        # GPS measurement is the noisy measurement of the vehicle's position  obtained from the simulator
        # GPS measurement is of the form [x,y,theta]

        self.R_gps = np.eye((3,3),dtype=np.float)

        y = gps_measurement - np.dot(self.H, self.x) #line use gps_measurement

        S_t = H_t @ P_t @ H_t.T + R_gps # mesurement residual covariance r gps

        K_t = P_t @ H_t.T @ np.linalg.pinv(S_t) # Kalman gain

        self.x = self.x + np.dot(K, y)

        I = np.eye(self.6) #np.ones for identity 

        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
                (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R_gps), K.T)



        # Output
        # Updates the state variable (self.x)

        # To do :
        # # Write code for measurement update 

        return 0


