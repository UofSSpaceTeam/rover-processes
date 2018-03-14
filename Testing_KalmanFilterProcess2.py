import numpy as np
import utm as utm
import statistics as stats
from robocluster import Device
import random as random

latitude = []
longitude = []

for index in range(1000):
    latitude.append(random.gauss(52.13255308, 0.0000699173))
    longitude.append(random.gauss(-106.6279284, 0.0000460515))


def create_covariance_of_process_noise(num_dimensions):
    # this is currently completely manual
    Q = np.eye(2 * num_dimensions)*0.005  # TODO: Figure out what this matrix actually is
    return Q

def create_covariance_of_observation_noise(num_dimensions):
    # this is currently completely manual
    # these values are taken from RoverGPSstatistics
    R = np.zeros((2 * num_dimensions, 2 * num_dimensions))
    R[0, 0] = 60.125
    R[1, 1] = 60.125
    R[0, 1] = 8.783
    R[1, 0] = 8.783
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
        H[index + 2, index + 2] = 0
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
            x_pp[index, 0] = initial_conditions[index]
    return x_pp

def create_control_inputs_vector(ax, ay):
    # takes accelerometer data and turns it into acceleration in northing easting and then creates a vector
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
    x_m = np.mat([[x_meas], [y_meas], [0], [0]])
    data2 = [x_meas, y_meas]
    zone_info = [zone_number, zone_letter]
    z = np.dot(H, x_m) + v
    return z, zone_info, data2

def update(num_dimensions, x_cp, P_cp, z, H, R):
    # takes in the (a priori) estimates along with the observations and appropriate matrices to calculate the true
    # state.
    I = np.eye(len(x_cp))  # Identity matrix needed for calculations
    y_p = z - np.dot(H, x_cp)  # measurement pre-fit residual
    S = R + np.dot(H, np.dot(P_cp, H.T)) # pre_fit residual covariance
    #S[S == 0] = 1
    K = np.dot(P_cp, H.T) / S  # Optimal Kalman gain
    K[np.isnan(K)] = 0
    K[np.isinf(K)] = 0

    x_cc = x_cp + np.dot(K, y_p)  # updated (a posteriori) state estimate
    L = (I - np.dot(K, H))  # intermediate calculation needed for later
    P_cc = np.dot(L, np.dot(P_cp, L.T)) + np.dot(K, np.dot(R, K.T)) # updated (a posteriori) estimate covariance
    P_cc[np.isnan(P_cc)] = 0
    y_c = z - np.dot(H, x_cc)  # measurement post-fit residual, we will probably never use this
    return x_cc, P_cc


    #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
dt = 0.1  # time step (can be determined by rate that GPS readings come in)
num_dimensions = 2  # number of dimensions in model
ax = 0
ay = 0
initialx = utm.from_latlon(52.13255308, -106.6279284)[0]
initialy = utm.from_latlon(52.13255308, -106.6279284)[1]
initial = [initialx, initialy]
F = create_state_transition_model(dt, num_dimensions)
B = create_control_input_model(dt, num_dimensions)
H = create_observation_model(num_dimensions)
x_pp = create_initial_state_vector(num_dimensions, initial)
P_pp = create_initial_process_error_covariance_matrix(num_dimensions)
Q = create_covariance_of_process_noise(num_dimensions)
R = create_covariance_of_observation_noise(num_dimensions)
v = create_observation_noise(num_dimensions)
u = create_control_inputs_vector(ax, ay)
accurate_lat = 0
accurate_long = 0
accurate_lat2 = 0
accurate_long2 = 0
trials1 = 0
filtered_gps_lat = []
filtered_gps_long = []

for index in range(len(latitude)):
    gps_latitude = latitude[index]
    gps_longitude = longitude[index]
    gps_latitude2 = utm.from_latlon(gps_latitude, gps_longitude)[0]
    gps_longitude2 = utm.from_latlon(gps_latitude, gps_longitude)[1]
    data = [gps_latitude, gps_longitude]
    [x_cp, P_cp] = predict(F, x_pp, B, u, P_pp, Q)
    [z, zone_info, data2] = observation(gps_latitude, gps_longitude, H, v)
    [x_cc, P_cc] = update(num_dimensions, x_cp, P_cp, z, H, R)
    true_gps = utm.to_latlon(x_cc[0, 0], x_cc[1, 0], zone_info[0], zone_info[1], strict=False)  # this has the value we
    true_gps2 = [x_cc[0, 0], x_cc[1, 0]]
    filtered_gps_lat.append(true_gps[0])
    filtered_gps_long.append(true_gps[1])
    pres = 2
    print('raw: {}'.format(data2))
    print("                                             filtered: {}".format(true_gps2))
    dif_lat = abs(true_gps2[0] - initial[0])
    dif_long = abs(true_gps2[1] - initial[1])
    if dif_lat < pres:
        print('Accurate Lat : ', dif_lat)
        accurate_lat += 1
    if dif_long < pres:
        print('Accurate Long : ', dif_long)
        accurate_long += 1

    trials1 += 1

    dif_lat = abs(gps_latitude2 - initial[0])
    dif_long = abs(gps_longitude2 - initial[1])
    if dif_lat < pres:
        print('Accurate Lat : ', dif_lat)
        accurate_lat2 += 1
    if dif_long < pres:
        print('Accurate Long : ', dif_long)
        accurate_long2 += 1



    x_pp = x_cc
    P_pp = P_cc

print('\nprecision: ', pres, 'm\n')

print('unfiltered: ')
print('accurate lat ', (accurate_lat2)/(trials1)*100,'%')
print('accurate long ', (accurate_long2)/(trials1)*100,'%')
print('variance of unfiltered lat, long: ', stats.variance(latitude), stats.variance(longitude))

print('\n')
print('filtered: ')
print('accurate lat ', (accurate_lat)/(trials1)*100,'%')
print('accurate long ', (accurate_long)/(trials1)*100,'%')
print('variance of filtered lat, long: ', stats.variance(filtered_gps_lat), stats.variance(filtered_gps_long))