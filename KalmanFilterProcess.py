import numpy as np
import utm as utm
from robocluster import Device
import random as random
import matplotlib.pyplot as plt
import time

KalmanFilter = Device('KalmanFilter', 'rover')

DummyGPS = Device('DummyGPS', 'rover')
@DummyGPS.every('0.1s')
async def dummy():
    #await DummyGPS.publish('singlePointGPS', [51.00000+(random.randrange(400, 600)/1000000), 110.00000+(random.randrange(600, 800)/1000000)])
    await DummyGPS.publish('singlePointGPS', [random.gauss(52.13255308, 0.0000699173), random.gauss(-106.6279284, 0.0000460515)])



@KalmanFilter.task
def start_up_kalman_filter():
    #User defined variables
    #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    dt = 0.1#time step (can be determined by rate that GPS readings come in)
    KalmanFilter.storage.dt = dt

    # GPS error
    KalmanFilter.storage.posX_Err_Meas = 5  # Error in position in x based on measurements (Error in GPS)
    KalmanFilter.storage.posY_Err_Meas = 5  # Error in position in y based on measurements (Error in GPS)
    KalmanFilter.storage.velX_Err_Meas = 0.2  # Error in velocity in x based on measurements (Error in GPS)
    KalmanFilter.storage.velY_Err_Meas = 0.2  # Error in velocity in y based on measurements (Error in GPS)

    KalmanFilter.storage.posX_mod_prev = 0  # Initial position in x (Model)
    KalmanFilter.storage.posY_mod_prev = 0  # Initial position in y (Model)
    KalmanFilter.storage.velX_mod_prev = 0  # Initial velocity in x (Model)
    KalmanFilter.storage.velY_mod_prev = 0  # Initial velocity in y (Model)

    KalmanFilter.storage.posX_Err = 1  # Error in x position based on model
    KalmanFilter.storage.posY_Err = 1  # Error in y position based on model
    KalmanFilter.storage.velX_Err = 0.2  # Error in x velocity based on model
    KalmanFilter.storage.velY_Err = 0.2  # Error in y velocity based on model

    KalmanFilter.storage.posX_meas_prev = 0
    KalmanFilter.storage.posY_meas_prev = 0

    KalmanFilter.storage.A = np.eye(4)
    # np.eye(n) creates identity matrix of nxn
    KalmanFilter.storage.A[0,2] = dt
    KalmanFilter.storage.A[1,3] = dt

    KalmanFilter.storage.B = np.mat([[0.5*(dt**2), 0], [0, 0.5*(dt**2)], [dt, 0], [0, dt]])

    q1_val = 0.003
    q2_val = 0.0003
    z1_val = 0.05
    z2_val = 0.005
    w1_val = 0.0002
    w2_val = 0.00002
    KalmanFilter.storage.pres = 0.0001
    KalmanFilter.storage.k = 0
    KalmanFilter.storage.noise_x = [0]*10000
    KalmanFilter.storage.noise_y = [0]*10000
    KalmanFilter.storage.filtered_x = [0]*10000
    KalmanFilter.storage.filtered_y = [0]*10000

    KalmanFilter.storage.Q = np.mat([[q1_val, 0, 0, 0], [0, q1_val, 0, 0], [0, 0, q2_val, 0], [0, 0, 0, q2_val]])

    KalmanFilter.storage.Z = np.mat([[z1_val], [z1_val], [z2_val], [z2_val]])  # Error/noise from electronics???

    KalmanFilter.storage.W = np.mat([[w1_val], [w1_val], [w2_val], [w2_val]])  # Predicted State Noise Matrix (Based off of Gaussian statistics??)

    KalmanFilter.storage.C = np.eye(4)

    KalmanFilter.storage.H = np.eye(4)  # Used to transform matrices into matrices of proper dimensions

    KalmanFilter.storage.X_prev = np.mat([[0], [0], [KalmanFilter.storage.velX_mod_prev], [KalmanFilter.storage.velY_mod_prev]])  # INITIAL Previous State Matrix (Model). Zeros are the initial position in x and y as we are always referencing from the current location

    KalmanFilter.storage.P_prev = np.mat([[KalmanFilter.storage.posX_Err**2, 0, 0, 0], [0, KalmanFilter.storage.posY_Err**2, 0, 0], [0, 0, KalmanFilter.storage.velX_Err**2, 0], [0, 0, 0, KalmanFilter.storage.velY_Err**2]])  # INITIAL Previous State Covariance Matrix (Error in Model)
    print(KalmanFilter.storage)



@KalmanFilter.on('*/singlePointGPS')
async def kalman_filter(event, data):
    A = KalmanFilter.storage.A
    B = KalmanFilter.storage.B
    Q = KalmanFilter.storage.Q
    Z = KalmanFilter.storage.Z
    W = KalmanFilter.storage.W
    C = KalmanFilter.storage.C
    H = KalmanFilter.storage.H
    dt = KalmanFilter.storage.dt
    P_prev = KalmanFilter.storage.P_prev
    X_prev = KalmanFilter.storage.X_prev
    pres = KalmanFilter.storage.pres
    k = KalmanFilter.storage.k
    noise_x = KalmanFilter.storage.noise_x
    noise_y = KalmanFilter.storage.noise_y
    filtered_x = KalmanFilter.storage.filtered_x
    filtered_y = KalmanFilter.storage.filtered_y

    '''posX_mod_prev = KalmanFilter.storage.posX_mod_prev
    posY_mod_prev = KalmanFilter.storage.posY_mod_prev
    velX_mod_prev = KalmanFilter.storage.velX_mod_prev
    velY_mod_prev = KalmanFilter.storage.velY_mod_prev'''

    posX_Err_Meas = KalmanFilter.storage.posX_Err_Meas
    posY_Err_Meas = KalmanFilter.storage.posY_Err_Meas
    velX_Err_Meas = KalmanFilter.storage.velX_Err_Meas
    velY_Err_Meas = KalmanFilter.storage.velY_Err_Meas

    '''posX_Err = KalmanFilter.storage.posX_Err
    posY_Err = KalmanFilter.storage.posY_Err
    velX_Err = KalmanFilter.storage.velX_Err
    velY_Err = KalmanFilter.storage.velY_Err'''

    posX_meas_prev = KalmanFilter.storage.posX_meas_prev
    posY_meas_prev = KalmanFilter.storage.posY_meas_prev

    latitude = data[0]
    longitude = data[1]

    #Sensor defined variables
    #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Accelerometer
    ax =  0# TODO: acceleration in x from accelerometer
    ay =  0# TODO: acceleration in x from accelerometer
    # GPS
    posX_meas_new =  utm.from_latlon(latitude, longitude)[0]  # position in x (Measured)(GPS)
    posY_meas_new =  utm.from_latlon(latitude, longitude)[1]  # position in y (Measured)(GPS)


    posX_meas =  posX_meas_new - posX_meas_prev # position in x (Measured)(GPS)
    posY_meas =  posY_meas_new - posY_meas_prev  # position in y (Measured)(GPS)
    velX_meas =  posX_meas/dt  # velocity in x (Measured)(GPS)
    velY_meas =  posY_meas/dt  # velocity in y (Measured)(GPS)
    zone_number = utm.from_latlon(latitude, longitude)[2]
    zone_letter = utm.from_latlon(latitude, longitude)[3]
    #print('zone number: ', zone_number, '\nzone letter:', zone_letter)

    R = np.mat([[posX_Err_Meas**2, 0, 0, 0],[0, posY_Err_Meas**2, 0, 0], [0, 0, velX_Err_Meas**2, 0], [0, 0, 0, velY_Err_Meas**2]])  # Measurement Covariance Matrix (Error in measurement) Error in gps readings
    u = np.mat([[ax], [ay]])  # Control variable matrix (where the accelerometer readings would go)
    Y_raw = np.matrix([[posX_meas], [posY_meas], [velX_meas], [velY_meas]])  # Raw Measurement
    Y = (C * Y_raw) + Z  # Measured Values used



    # Process Noise Covariance (Based off of Gaussian statistics?) Error from things unaccounted for in our physical model

    P_predic = (A * P_prev * np.transpose(A)) + Q  # Current State Covariance Matrix (Error in Model)
    P_predic[0, 1:3] = 0
    P_predic[1, 0] = 0
    P_predic[1, 2:4] = 0
    P_predic[2, 3] = 0
    P_predic[2, 0:1] = 0
    P_predic[3, 0:2] = 0



    X_predic = (A * X_prev) + (B * u) + W  # State Matrix (Model)

    Y[2, 0] = X_predic[2, 0]
    Y[3, 0] = X_predic[3, 0]
    #print('Y:\n', Y)


    K = (P_predic * np.transpose(H))/((H * P_predic * np.transpose(H)) + R)
    K[0, 1:4] = 0
    K[1, 0] = 0
    K[1, 2:4] = 0
    K[2, 3] = 0
    K[2, 0:2] = 0
    K[3, 0:3] = 0
    #print('K')
    #print(K)
    #print('\n')
    X = X_predic + (K * (Y - (H * X_predic)))  # "Actual" state
    P = (np.eye(4) - (K * H)) * P_predic  # "Actual" process Covariance Matrix
    #print('P')
    #print(P)
    #print('\n')

    '''x_vel=X[2,0]
    y_vel=X[3,0]'''
    #print("\nlat:", X[0, 0] + posX_meas_prev, "\nlong:", X[1, 0] + posY_meas_prev)
    true_gps = utm.to_latlon(X[0, 0] + posX_meas_prev, X[1, 0] + posY_meas_prev, zone_number, zone_letter, strict=False)  # this has the value we want
    #print(X[0, 0] + posX_meas_prev)
    #print(X[1, 0] + posY_meas_prev)
    print('raw: {}'.format(data))
    print("                                             filtered: {}".format(true_gps))
    dif_lat = abs(true_gps[0] - 51.000000)
    dif_long = abs(true_gps[1] - 110.000000)
    if dif_lat < pres and dif_long < pres:
        print('Accurate: ', dif_lat, ', ', dif_long)
    else:
        print('Try Again: ', dif_lat, ', ', dif_long)
    '''print('X_predic:\n', X_predic)
    print('Y:\n', Y)
    print('X:\n', X)
    print('K:\n', K)'''


    X[0, 0] = 0
    X[1, 0] = 0
    '''X[2, 0] = 0
    X[3, 0] = 0'''

    #print('X:\n', X)

    noise_x[k] = data[0]
    noise_y[k] = data[1]
    filtered_x[k] = true_gps[0]
    filtered_y[k] = true_gps[1]
    k += 1




    await KalmanFilter.publish('FilteredGPS', true_gps)
    KalmanFilter.storage.posX_meas_prev = posX_meas_new
    KalmanFilter.storage.posY_meas_prev = posY_meas_new
    KalmanFilter.storage.X_prev = X  # Set current to the new previous for next cycle
    #KalmanFilter.storage.X_prev[0, 0] = 0  # so that x is always dx
    #KalmanFilter.storage.X_prev[1, 0] = 0  # so that y is always dy
    KalmanFilter.storage.P_prev = P  # Set current to the new previous for next cycle
    KalmanFilter.storage.k = k
    KalmanFilter.storage.noise_x = noise_x
    KalmanFilter.storage.noise_y = noise_y
    KalmanFilter.storage.filtered_x = filtered_x
    KalmanFilter.storage.filtered_y = filtered_y



try:
    KalmanFilter.start()
    DummyGPS.start()
    time.sleep(35)
    KalmanFilter.stop()
    for i in range(0,350):
        print("{},{}".format(KalmanFilter.storage.filtered_x[i], KalmanFilter.storage.filtered_y[i]))
    print("New Data\n\n")
    for i in range(0,350):
        print("{},{}".format(KalmanFilter.storage.noise_x[i], KalmanFilter.storage.noise_y[i]))

    DummyGPS.wait()
except KeyboardInterrupt:
    KalmanFilter.stop()
    DummyGPS.stop()
