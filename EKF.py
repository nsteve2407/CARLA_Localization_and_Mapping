import numpy as np



class EKF():
    def __init__(self):
        self.x = np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T # 6D state vector [x,y,theta,x',y',theta']
        # self.x = np.expand_dims(self.x,axis=-1)
        self.P = np.eye(6,dtype=np.float)
        self.Q = np.eye(6,dtype=np.float)
        self.R_lidar = np.eye(3,dtype=np.float)*0.5
        self.R_gps = np.eye(3,dtype=np.float)*2.0
        self.R_gps[2,2] = 0.3
        self.A = np.eye(6,dtype=np.float)
        self.H = np.array([[1.0,0.0,0.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0,0.0,0.0],[0.0,0.0,1.0,0.0,0.0,0.0]]) #6*6
        self.q = 1.0
        self.initialized  = False

    def predict(self,delta_t):
              
        self.A[0,3]=delta_t
        self.A[1,4]=delta_t
        self.A[2,5]=delta_t

        q1 = (delta_t**3*self.q)/3
        q2 = (delta_t**2*self.q)/2
        q3 = (delta_t*self.q)

        self.Q = np.array([[q1,0,0,q2,0,0],[0,q1,0,0,q2,0],[0,0,q1,0,0,q2],[q2,0,0,q3,0,0],[0,q2,0,0,q3,0],[0,0,q2,0,0,q3]])
        

        self.x = self.A @ self.x
        
        self.P = ((self.A @ self.P) @ self.A.T) + self.Q
        # To do
        # Implement constant velocity motion model to update state(self.x)

        #Update self.A matrix (A[0,3]=delta_t,A[1,4]=delta_t,A[2,5]=delta_t)

        #Update state (self.x = A@x)

        #Update self.P

        return 0
    
    def measurement_update_lidar(self,lidar_measurement):
        # Input:
        # Lidar measurement is the lidar position estimate obtained from Scan Matching
        # Lidar measurement is of the form [x,y,theta]
       

       y = lidar_measurement - np.dot(self.H, self.x) #line 27 for z lidar mesa

       S_t = (self.H @ self.P) @ self.H.T + self.R_lidar # mesurement residual covariance r lidar

       K = (self.P @ self.H.T) @ np.linalg.pinv(S_t) # Kalman gain

       self.x = self.x + np.dot(K, y)

       I = np.eye(6,6) #np.ones

       self.P = np.dot(I - np.dot(K, self.H), self.P)
        # Output
        # Updates the state variable (self.x)
       return 0
    def measurement_update_gps(self,gps_measurement):
        # Input:
        # GPS measurement is the noisy measurement of the vehicle's position  obtained from the simulator
        # GPS measurement is of the form [x,y,theta]

       y = gps_measurement - np.dot(self.H, self.x) #line use gps_measurement

       S_t = ((self.H @ self.P) @ self.H.T) + self.R_gps # mesurement residual covariance r gps

       K = (self.P @ self.H.T) @ np.linalg.pinv(S_t) # Kalman gain

       self.x = self.x + np.dot(K, y)

       I = np.eye(6) #np.ones for identity 

       self.P = np.dot(I - np.dot(K, self.H), self.P)



        # Output
        # Updates the state variable (self.x)

        # To do :
        # # Write code for measurement update 
       return 0