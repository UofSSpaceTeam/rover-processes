import numpy as np
import matplotlib as mpl


#User defined variables
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
dt = 0.01#time step (can be determined by rate that GPS readings come in)


#Sensor defined variables
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#Accelerometer
ax = 0.1#acceleration in x from accelerometer
ay = 0.1#acceleration in x from accelerometer
#GPS
posX_meas = 10  # position in x (Measured)(GPS)
posY_meas = 10  # position in y (Measured)(GPS)
velX_meas = 1  # velocity in x (Measured)(GPS)
velY_meas = 1  # velocity in y (Measured)(GPS)
#GPS error
posXErrMeas = 3  # Error in position in x based on measurements (Error in GPS)
posYErrMeas = 3  # Error in position in y based on measurements (Error in GPS)
velXErrMeas = 1  # Error in velocity in x based on measurements (Error in GPS)
velYErrMeas = 1  # Error in velocity in y based on measurements (Error in GPS)


#Matrices and variables from measurements
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Z = np.mat([[0], [0], [0], [0]])  # Error/noise from electronics???
R = np.mat([[posXErrMeas**2, 0, 0, 0],[0, posYErrMeas**2, 0, 0], [0, 0, velXErrMeas**2, 0], [0, 0, 0, velYErrMeas**2]])  # Measurement Covariance Matrix (Error in measurement) Error in gps readings
C = np.eye(4)

Y_raw = np.matrix([[posX_meas], [posY_meas], [velX_meas], [velY_meas]])  # Raw Measurement
Y = C * Y_raw + Z  # Measured Values used


#Matrices and variables for model
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
A = np.eye(4)
#np.eye(n) creates identity matrix of nxn
A[0,2] = dt
A[1,3] = dt

B = np.mat([[0.5*(dt**2), 0], [0, 0.5*(dt**2)], [dt, 0], [0, dt]])

posX_mod_prev = 0  # Initial position in x (Model)
posY_mod_prev = 0  # Initial position in y (Model)
velX_mod_prev = 0  # Initial velocity in x (Model)
velY_mod_prev = 0  # Initial velocity in y (Model)

posXErr = 1.5  # Error in x position based on model
posYErr = 1.5  # Error in y position based on model
velXErr = 0.5  # Error in x velocity based on model
velYErr = 0.5  # Error in y velocity based on model

u = np.mat([[ax],[ay]])  # Control variable matrix (where the accelerometer readings would go)

Q = np.mat([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])  # Process Noise Covariance (Based off of Gaussian statistics?) Error from things unaccounted for in our physical model

P_prev = np.mat([[posXErr**2, 0, 0, 0], [0, posYErr**2, 0, 0], [0, 0, velXErr**2, 0], [0, 0, 0, velYErr**2]])  # INITIAL Previous State Covariance Matrix (Error in Model)

P_predic = A * P_prev * np.transpose(A) + Q  # Current State Covariance Matrix (Error in Model)
P_predic[0, 1:3] = 0
P_predic[1, 0] = 0
P_predic[1, 2:4] = 0
P_predic[2, 3] = 0
P_predic[2, 0:1] = 0
P_predic[3, 0:2] = 0

W = np.mat([[0], [0], [0], [0]])  # Predicted State Noise Matrix (Based off of Gaussian statistics??)

X_prev = np.mat([[posX_mod_prev], [posY_mod_prev], [velX_mod_prev], [velY_mod_prev]])  # INITIAL Previous State Matrix (Model)

X_predic = (A * X_prev) + (B * u) + W  # State Matrix (Model)


#Kalman gain and current state
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
H = np.eye(4)  # Used to transform matrices into matrices of proper dimensions

K = (P_predic * np.transpose(H))/((H * P_predic * np.transpose(H)) + R)
K[0, 1:4] = 0
K[1, 0] = 0
K[1, 2:4] = 0
K[2, 3] = 0
K[2, 0:2] = 0
K[3, 0:3] = 0

X = X_predic + K * (Y - H * X_predic)  # "Actual" state
P = (np.eye(4) - K * H) * P_predic  # "Actual" process Covariance Matrix

X_prev = X  # Set current to the new previous for next cycle
P_prev = P  # Set current to the new previous for next cycle



'''
print('A')
print(A)
print('\n')
print('B')
print(B)
print('\n')
print('C')
print(C)
print('\n')
print('Previous Process Covariance')
print(P_prev)
print('\n')
print('Predicted Process Covariance')
print(P_predic)
print('\n')
print('Previous State')
print(X_prev)
print('\n')
print('B')
print(B)
print('\n')
print('Control Variable Matrix')
print(u)
print('\n')
print('Predicted Current State')
print(X_predic)
print('\n')
print('Error in measurement R')
print(R)
print('\n')
print('Kalman Gain')
print(K)
print('\n')
print('Measured Values')
print(Y)
print('\n')'''
print('"Actual" state')
print(X)
print('\n')
print('"Actual" Process Covariance')
print(P)
print('\n')