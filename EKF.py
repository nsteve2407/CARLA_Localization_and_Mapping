import numpy as np



class EKF():
    def __init__(self):
        self.x = np.array[0.0,0.0,0.0,0.0,0.0,0.0] # 6D state vector [x,y,theta,x',y',theta']
        self.P = np.eye((6,6),dtype=np.float)
        self.Q = np.eye((6,6),dtype=np.float)
        self.R_lidar = np.eye((3,3),dtype=np.float)


    def predict(self,delta_t,state_estimate_t_minus_1,P_t_minus_1,theta,z_t_observation_vector): 

        # Inputs

        control_vector_t_minus_1 = np.array([5,0.0])  #control constant velocity in x direction (5m/s)

        Q_t = np.array([[1.0,   0,   0],[  0, 1.0,   0],[  0,   0, 1.0]]) # state model noise covariance

        H_t = np.array([[1.0,  0,   0],[  0,1.0,   0],[  0,  0, 1.0]]) #H martix used to convert the predicted state estimate at time t into predicted sensor measurements at time t

        R_t = np.array([[1.0,   0,    0],[  0, 1.0,    0],[  0,    0, 1.0]])  # Sensor covariance noise

        B = np.array([  [np.cos(theta)*delta_t, 0],[np.sin(theta)*delta_t, 0],[0, delta_t]])

        sensor_noise_w_t = np.array([0.07,0.07,0.04])


        # State transition matrix A at timestep t-1
        
        A_t_minus_1 = np.array([[1.0,  0,   0],[  0,1.0,   0],[  0,  0, 1.0]])


        # Covariance
        
        P_t = A_t_minus_1 @ P_t_minus_1 @ A_t_minus_1.T + (Q_t)

        # takes input as the delta time betweem previous state estiate and current estimate
        
        state_estimate_t = A_t_minus_1 @ (state_estimate_t_minus_1) + (B(state_estimate_t_minus_1[2],delta_t)) @ (control_vector_t_minus_1) 
        
        
        # Outputs
        
        measurement_residual_y_t = z_t_observation_vector - ((H_t @ state_estimate_t) + (sensor_noise_w_t))

        S_t = H_t @ P_t @ H_t.T + R_t # mesurement residual covariance 

        K_t = P_t @ H_t.T @ np.linalg.pinv(S_t) # Kalman gain

        # Updates state(self.x)
        state_estimate_t = state_estimate_t + (K_t @ measurement_residual_y_t)

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


