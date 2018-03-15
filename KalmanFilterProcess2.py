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


def create_covariance_of_process_noise(num_dimensions):
    # this is currently completely manual
    Q = np.eye(2 * num_dimensions)  # TODO: Figure out what this matrix actually is
    return Q

def create_covariance_of_observation_noise(num_dimensions):
    # this is currently completely manual
    R = np.zeros((2 * num_dimensions, 2 * num_dimensions))
    R[0, 0] = 60.125
    R[2, 2] = 60.125#10.308
    R[0, 2] = 8.783
    R[2, 0] = 8.783
    return R

def create_observation_noise(num_dimensions):
    # this is currently completely manual
    # drawn from zero mean white noise gaussian with covariance, R:v ~ N(0,R)
    v = np.zeros((2 * num_dimensions, 1))  # TODO: Figure out what this vector/matrix is
    v[0, 0] = 0.001
    v[1, 0] = 0.001
    return v

def create_state_transition_model(time_step, num_dimensions):
    # takes a time step value and a number of dimensions and returns a state_transition model of the form
    # pos1, pos2, ..., posn, vel1, vel2, ..., veln. This is opposed to a model of the form pos1, vel1, pos2, vel2...
    F = np.eye(2 * num_dimensions)
    for index in range(num_dimensions):
        F[index, index + 2] = time_step
    return F

def create_control_input_model(time_step, num_dimensions):
    # takes a time step value and a number of dimensions and returns a control input model of the form
    # pos1, pos2, ..., posn, vel1, vel2, ..., veln. This is opposed to a model of the form pos1, vel1, pos2, vel2...
    B = np.zeros((2 * num_dimensions, num_dimensions))
    for index in range(num_dimensions):
        B[index, index] = 0.5 * (time_step ** 2)
        B[index + 2, index] = time_step
    return B

def create_observation_model(num_dimensions):
    # takes the number of dimensions and creates the observation matrix used to map the true state to the observed
    # space. For our uses this is simply an identity matrix.
    H = np.eye(2 * num_dimensions)
    for index in range(num_dimensions):
        H[(2 * index) + 1, (2 * index) +1] = 0
    return H

def create_initial_process_error_covariance_matrix(num_dimensions, variance=0):
    # takes in the number of dimensions and the variable variance values and creates an initial process error
    # covariance matrix. If initial positions are known to a decent level of accuracy this is just a zero matrix
    # hence the default variance of 0.
    # If initial positions are not known then variance should be a list of variances of the form
    # pos1, pos2, ..., posn, vel1, vel2, ..., veln. This is opposed to a model of the form pos1, vel1, pos2, vel2...

    P_pp = np.zeros((2 * num_dimensions, 2 * num_dimensions))
    if variance != 0:
        for index in range(2 * num_dimensions):
            P_pp[index, index] = variance[index]
    return P_pp

def create_initial_state_vector(num_dimensions, initial_conditions=0):
    # takes in the number of dimensions and the initial conditions and creates an initial state vector.
    # If initial positions are chosen to be entered then they should be in a list of the form
    # pos1, pos2, ..., posn, vel1, vel2, ..., veln. This is opposed to a model of the form pos1, vel1, pos2, vel2...
    # Initial positions should be entered but are optional. Ideally if no initial conditions are not entered then
    # the variance in the initial process error covariance matrix should have npne default variance parameters.
    # Don't believe this is actually too important though
    x_pp = np.zeros((2 * num_dimensions, 1))
    if initial_conditions != 0:
        for index in range(num_dimensions):
            x_pp[index, 1] = initial_conditions[index]
    return x_pp

def create_control_inputs_vector(ax, ay):
    # takes accelerometer data and turns it into acceleration in northing easting and then creates a vector
    u = np.array([[ax],[ay]])
    u = np.zeros((2, 1))  # TODO: actually calculate acceleration in northing easting and create vector
    return u

def predict(F, x_pp, B, u, P_pp, Q):
    # takes the previous (a posteriori) state estimate and previous (a posteriori) process error covariance matrix
    # and creates the predicted (a priori) state estimate and the predicted (a priori) estimate covariance
    x_cp = np.dot(F, x_pp) + np.dot(B, u)
    P_cp = np.dot(F, np.dot(P_pp, F.T)) + Q
    return x_cp, P_cp

def observation(gps_latitude, gps_longitude, H, v):
    # currently only for 2d system
    # takes GPS measurements and converts them to northing and easting values.
    [x_meas, y_meas, zone_number, zone_letter] = utm.from_latlon(gps_latitude, gps_longitude)
    x_m = np.mat([[x_meas], [0], [y_meas], [0]])
    zone_info = [zone_number, zone_letter]
    z = np.dot(H, x_m) + v
    return z, zone_info

def update(num_dimensions, x_cp, P_cp, z, H, R):
    # takes in the (a priori) estimates along with the observations and appropriate matrices to calculate the true
    # state.
    I = np.eye(len(x_cp))  # Identity matrix needed for calculations
    y_p = z - np.dot(H, x_cp)  # measurement pre-fit residual
    S = R + np.dot(H, np.dot(P_cp, H.T)) # pre_fit residual covariance
    S[S == 0] = 1
    K = np.dot(P_cp, H.T) / S  # Optimal Kalman gain

    K[np.isnan(K)] = 0

    x_cc = x_cp + np.dot(K, y_p)  # updated (a posteriori) state estimate
    L = (I - np.dot(K, H))  # intermediate calculation needed for later
    P_cc = np.dot(L, np.dot(P_cp, L.T)) + np.dot(K, np.dot(R, K.T)) # updated (a posteriori) estimate covariance
    y_c = z - np.dot(H, x_cc)  # measurement post-fit residual, we will probably never use this
    return x_cc, P_cc


@KalmanFilter.task
def start_up_kalman_filter():
    #User defined variables
    #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    dt = 0.1  # time step (can be determined by rate that GPS readings come in)
    num_dimensions = 2  # number of dimensions in model
    ax = 0
    ay = 0
    KalmanFilter.storage.dt = dt
    KalmanFilter.storage.F = create_state_transition_model(dt, num_dimensions)
    KalmanFilter.storage.B = create_control_input_model(dt, num_dimensions)
    KalmanFilter.storage.H = create_observation_model(num_dimensions)
    KalmanFilter.storage.x_pp = create_initial_state_vector(num_dimensions)
    KalmanFilter.storage.P_pp = create_initial_process_error_covariance_matrix(num_dimensions)
    KalmanFilter.storage.Q = create_covariance_of_process_noise(num_dimensions)
    KalmanFilter.storage.R = create_covariance_of_observation_noise(num_dimensions)
    KalmanFilter.storage.v = create_observation_noise(num_dimensions)
    KalmanFilter.storage.u = create_control_inputs_vector(ax, ay)



@KalmanFilter.on('*/singlePointGPS')
async def kalman_filter(event, data):
    num_dimensions = 2
    F = KalmanFilter.storage.F
    B = KalmanFilter.storage.B
    H = KalmanFilter.storage.H
    x_pp = KalmanFilter.storage.x_pp
    P_pp = KalmanFilter.storage.P_pp
    Q = KalmanFilter.storage.Q
    R = KalmanFilter.storage.R
    v = KalmanFilter.storage.v
    u = KalmanFilter.storage.u
    # u = create_control_inputs_vector(ax, ay) is where the accelerometer values go
    gps_latitude = data[0]
    gps_longitude = data[1]
    [x_cp, P_cp] = predict(F, x_pp, B, u, P_pp, Q)
    [z, zone_info] = observation(gps_latitude, gps_longitude, H, v)  # This is where gps values are inputted
    [x_cc, P_cc] = update(num_dimensions, x_cp, P_cp, z, H, R)
    true_gps = utm.to_latlon(x_cc[0, 0], x_cc[1, 0], zone_info[0], zone_info[1], strict=False)  # this has the value we

    pres = 0.00001
    print('raw: {}'.format(data))
    print("                                             filtered: {}".format(true_gps))
    dif_lat = abs(true_gps[0] - 51.000000)
    dif_long = abs(true_gps[1] - 110.000000)
    if dif_lat < pres and dif_long < pres:
        print('Accurate: ', dif_lat, ', ', dif_long)
    else:
        print('Try Again: ', dif_lat, ', ', dif_long)



    await KalmanFilter.publish('FilteredGPS', true_gps)
    KalmanFilter.storage.x_pp = x_cc
    KalmanFilter.storage.P_pp = P_cc


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