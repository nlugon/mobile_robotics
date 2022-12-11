'''
Extended Kalman Filter class

Code modified from https://github.com/L42Project/Tutoriels/blob/master/Divers/tutoriel36/KalmanFilter.py
Comments describing Kalman Filter taken from https://stackoverflow.com/questions/74318200/how-to-tune-extended-kalman-filter-on-pykalman

Example of instance: KF=KalmanFilter(0.03333, [0, 0, 0, 0, 0]) (dt = 0.033 is the time between each measurement, x_ini = [0, 0, 0, 0, 0] is the initial state)

For update() function : takes as input the measurement vector z
z[0:2] are the measurements from the camera (px, py, orientation)
z[3:4] are the measurements from the Thymio (forward speed v and angular speed omega)

When Thymio position not found by camera -> z becomes 2x1 vector, with z[0:1] being the forward speed v and angular speed omega
'''
import numpy as np

class KalmanFilter():
    def __init__(self, dt, x_ini):
        self.dt = dt

        # Initial State Vector
        self.x=np.matrix([[x_ini[0]], [x_ini[1]], [x_ini[2]], [x_ini[3]], [x_ini[4]]])
    

        # Initialize State Transition Matrix (Warning : changes with time -> updateFk())
        self.Fk=np.eye(5)

        # Initialize Measurement matrix H
        # Used to convert the predicted state estimate into predicted sensor measurements at time k.
        # In this case, H will be the identity matrix since the estimated state maps directly to state measurements data
        # H has the same number of rows as sensor measurements and same number of columns as states.
        self.H=np.eye(5)

        
        # State model noise covariance matrix Q
        # When Q is large, the Kalman Filter tracks large changes in
        # the sensor measurements more closely than for smaller Q.
        # Q is a square matrix that has the same number of rows as states.
        self.Q=np.diag([1, 1, 0.1, 1, 0.1])  
        

        # Sensor measurement noise covariance matrix R
        # Has the same number of rows and columns as sensor measurements.
        # If we are sure about the measurements, R will be near zero.
        self.coord_var = 0.01
        self.angle_var = 0.001
        self.speed_var = 1
        self.omega_var = 0.1
        
        self.R=np.diag([self.coord_var, self.coord_var, self.angle_var, self.speed_var, self.omega_var])

        self.P=np.eye(5)
        
    def updateFk(self):
        '''
        Function that updates the state transition matrix Fk
        '''
        
        self.Fk=np.matrix([[1.0, 0, 0, self.dt*math.cos(self.x[2]), 0],
                           [0, 1.0,  0, self.dt*math.sin(self.x[2]), 0],
                           [0, 0, 1.0, 0, self.dt],
                           [0, 0, 0, 1.0, 0],
                           [0, 0, 0, 0, 1.0]])
        self.Fk = self.Fk.astype(float)

    def predict(self):
        # Update Fk
        self.updateFk()
        # Predict the state estimate (A Priori) at time k based on the state estimate at time k-1
        self.x=np.dot(self.Fk, self.x)
        # Predict the state covariance estimate based on the previous covariance and some noise
        self.P=np.dot(np.dot(self.Fk, self.P), self.Fk.T)+self.Q
        return self.x

    def update(self, z, CameraAccessible = True): # z[0:2] corrpesonds to measurement of camera, z[3:4] corresponds to measurements of wheels
        
        if CameraAccessible:
            self.H=np.eye(5)
            
            self.R=np.diag([self.coord_var, self.coord_var, self.angle_var, self.speed_var, self.omega_var])
        else: # CAUTION : IF CAMERAACCESSIBLE = FALSE, Z IS A 2x1 VECTOR (z[0:1] are now measrurements from wheels)
            # Measurement matrix H (now only v and omega can be observed)
            self.H=np.matrix([[0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 1]])
            
            self.R=np.diag([self.speed_var, self.omega_var])
            
            
        # Compute Kalman gain
        S=np.dot(self.H, np.dot(self.P, self.H.T))+self.R
        inv_S = np.linalg.pinv(S.astype(float))
        K=np.dot(np.dot(self.P, self.H.T),inv_S)

        # Correction / innovation
        # Calculate an updated state estimate (A Posteriori) for time k
        self.x=self.x+np.dot(K, (z-np.dot(self.H, self.x)))
        # Update the state covariance estimate for time k
        I=np.eye(self.H.shape[1])
        self.P=(I-(K*self.H))*self.P


        return self.x