from .GPSdriver import GPSPosition, LOOP_PERIOD
from math import asin, atan2, cos, pi, radians, sin, sqrt, degrees, atan
import time
#from statistics import mean
from .differential_drive_lib import diff_drive_fk, inverse_kinematics_drive
import json
import math
from .lidarmap import LidarMap
from robocluster import device

'''
 Aggregates data from GPS, Magnetometer, LIDAR, etc,
 and makes driving decisions based on the rover's
 surroundings.
'''

WHEEL_RADIUS = 14.5 # cm
MIN_WHEEL_RPM = 4.385095 # ERPM = 1000

ROVER_WIDTH = 1.2 # m

CALIBRATION_SAMPLES = 10

LIDAR_ANGLE_UNIT = 0.6
LIDAR_POINTS = 600
LIDAR_MAX_RANGE = 600


NavDevice = Device('NavDevice', 'demo-device')

# Initialize setup variables
NavDevice.storage.position = None
NavDevice.storage.position_last = None

NavDevice.storage.heading = 0
NavDevice.storage.heading_last = 0

NavDevice.storage.velocity = [0,0] # m/s, north, east
NavDevice.storage.accel = [0,0] # m/s^2, north, east

NavDevice.storage.bearing_error = 5 # degrees
NavDevice.storage._rotating = False

NavDevice.storage.autonomous_mode = True
NavDevice.storage.state = "manual" # can be "waiting", "driving", or "manual"

# number of samples in our running average
NavDevice.storage.pos_samples = 0
NavDevice.storage.vel_samples = 0
NavDevice.storage.heading_samples = 0

NavDevice.storage.last_compassmessage = 0

NavDevice.storage.target = None
NavDevice.storage.target_reached_distance = 3  # metres
NavDevice.storage.target_maximum_distance = 10 # metres

# Delay for loop in miliseconds
NavDevice.storage.loop_delay = 0.1

# TODO: Why is this 2000, I found it in DriveProcess.py as min_rpm?
NavDevice.storage.motor_rpm = 2000
NavDevice.storage.right_rpm = 0 # wheel RPMs
NavDevice.storage.left_rpm = 0 # wheel RPMs
NavDevice.storage.right_speed = 0 # velocity of right wheel from axel
NavDevice.storage.left_speed = 0 # velocity of left wheel from axel

NavDevice.storage.starting_calibration_gps = [[],[]] # list of GPS positions that ar averaged
NavDevice.storage.starting_calibration_heading = []

NavDevice.storage.waypoints = []

NavDevice.storage.lidar_angles = [i * LIDAR_ANGLE_UNIT for i in range(0, LIDAR_POINTS)]
NavDevice.storage.lidar_distance = [LIDAR_MAX_RANGE for i in range(0, LIDAR_POINTS)]
NavDevice.storage.lidar_scan_finish = False

@NavDevice.every(NavDevice.storage.loop_delay)
async def every():
    if NavDevice.storage.state == "waiting":
		NavDevice.wait_state()
	elif NavDevice.storage.state == "driving":
		NavDevice.drive_state()
	elif NavDevice.storage.state == "manual":
		pass # don't do anything in manual control mode
	else:
		.state = "waiting"
	NavDevice.update_wheel_velocity()

async def drive_state():
	''' Function for handling drive state '''
	if  NavDevice.storage.position is not None:
		distance =  NavDevice.storage.position.distance(NavDevice.storage.target)
		bearing = NavDevice.storage.position.bearing(NavDevice.storage.target)
		difference = bearing - NavDevice.storage.heading
		if abs(difference) > NavDevice.storage.bearing_error:
			if difference < 0:
				await NavDevice.publish("DriveRotateLeft", MIN_WHEEL_RPM)
				NavDevice.storage.left_rpm = -MIN_WHEEL_RPM
				NavDevice.storage.right_rpm = MIN_WHEEL_RPM
			else:
				await NavDevice.publish("DriveRotateRight", MIN_WHEEL_RPM)
				NavDevice.storage.left_rpm = MIN_WHEEL_RPM
				NavDevice.storage.right_rpm = -MIN_WHEEL_RPM
			return # back to loop
		elif distance > NavDevice.storage.target_reached_distance:
			await NavDevice.publish("DriveForward", MIN_WHEEL_RPM)
			NavDevice.storage.left_rpm = MIN_WHEEL_RPM
			NavDevice.storage.right_rpm = MIN_WHEEL_RPM
			return #back to loop
		else:
			NavDevice.storage.target = None
			await NavDevice.publish("TargetReached")
			NavDevice.storage.state = "waiting"

async def wait_state():
	''' Function for handling waiting state '''
	if NavDevice.storage.autonomous_mode:
		if len(NavDevice.storage.waypoints) == 0:
			await NavDevice.publish("DriveStop", 0)
	else:
		await NavDevice.publish("DriveStop", 0)

def update_wheel_velocity():
	NavDevice.storage.right_speed = NavDevice.storage.right_rpm*WHEEL_RADIUS/2
	NavDevice.storage.left_speed = NavDevice.storage.left_rpm*WHEEL_RADIUS/2

@NavDevice.on('CompassDataMessage')
async def CompassDataMessage_callback('CompassDataMessage', msg):
	''' CompassDataMessage contains:
		heading (degrees): Relative to north, the angle of
			rotation on the axis normal to the earth's surface.
		pitch (degrees):
		roll (degrees):
	'''
	if NavDevice.storage.heading is not None:
		NavDevice.storage.heading_last = NavDevice.storage.heading
		if NavDevice.storage.state == "waiting":
			# rolling average
			NavDevice.storage.heading = (NavDevice.storage.heading_samples*NavDevice.storage.heading + msg.heading)/(NavDevice.storage.heading_samples+1)
			NavDevice.storage.heading_samples += 1
		else:
			tmp = time.time()
			d_t = tmp-NavDevice.storage.last_compassmessage
			NavDevice.storage.last_compassmessage = tmp
			NavDevice.storage.heading = NavDevice.g_h_filter(msg.heading, NavDevice.storage.heading,
					(NavDevice.storage.right_speed-NavDevice.storage.left_speed)/ROVER_WIDTH, 0.8, d_t)
			await NavDevice.publish("RoverHeading", NavDevice.storage.heading)
		# self.log("heading: {}".format(self.heading))

@NavDevice.on('targetGPS')
async def targetGPS_callback('targetGPS', pos):
	''' Targets a new GPS coordinate '''
	target = GPSPosition(radians(pos[0]), radians(pos[1]))
	await NavDevice.publish("TargetReached", False)
	if target.distance(NavDevice.storage.position) <= NavDevice.storage.maximum_target_distance:
		if len(NavDevice.storage.waypoints) > 1:
			NavDevice.storage.waypoints.append(target)
			NavDevice.storage.target = waypoints[0]
		else:
			NavDevice.storage.target = target
		if NavDevice.storage.state != "manual":
			NavDevice.storage.state = "driving"

@NavDevice.on('singlePointGPS')
async def singlePointGPS_callback('singlePointGPS', pos):
	''' Updates GPS position '''
	#std_dev 1.663596084712623e-05, 2.1743680968892167e-05
	# self.log("{},{}".format(degrees(pos.lat), degrees(pos.lon)))
	if NavDevice.storage.state == "waiting" and NavDevice.storage.position is not None:
		lat = (NavDevice.storage.position.lat + pos.lat)/(NavDevice.storage.pos_samples)
		lon = (NavDevice.storage.pos_samples*NavDevice.storage.position.lon + pos.lon)/(NavDevice.storage.pos_samples+1)
		NavDevice.storage.pos_samples += 1
		await NavDevice.publish("RoverPosition", [degrees(NavDevice.storage.position.lat), degrees(NavDevice.storage.position.lon)])
		return
	if NavDevice.storage.position is not None:
		NavDevice.storage.position_last = NavDevice.storage.position
		NavDevice.storage.position = pos
		await NavDevice.publish("RoverPosition", [degrees(pos.lat), degrees(pos.lon)])
	else:
		if len(NavDevice.storage.starting_calibration_gps[0]) < CALIBRATION_SAMPLES:
			NavDevice.storage.starting_calibration_gps[0].append(pos.lat)
			NavDevice.storage.starting_calibration_gps[1].append(pos.lon)
		else:
			# Done averaging
			NavDevice.storage.starting_calibration_gps[0].append(pos.lat)
			NavDevice.storage.starting_calibration_gps[1].append(pos.lon)
			NavDevice.storage.position = GPSPosition(mean(NavDevice.storage.starting_calibration_gps[0]),
				mean(NavDevice.storage.starting_calibration_gps[1]))
			#self.log("{},{}".format(degrees(pos.lat), degrees(pos.lon)), "INFO")

@NavDevice.on('GPSVelocity')
def GPSVelocity_callback('GPSVelocity', vel):
	''' Updates velocity from GPS unit '''
	# std_dev 0.04679680341613995, 0.035958365746391524
	# self.log("{},{}".format(vel[0], vel[1]))
	NavDevice.storage.velocity = vel


@NavDevice.on('ButtonA_down')
def ButtonA_down_callback('ButtonA_down', data):
	''' Go into the "waiting" state '''
	NavDevice.storage.state = "waiting"

@NavDevice.on('ButtonB_down')
def ButtonB_down_callback('ButtonB_down', data):
	''' Go into manual control mode '''
	NavDevice.storage.state = "manual"

@NavDevice.on('ButtonY_down')
def ButtonY_down_callback('ButtonY_down', pos):
	''' Add the rover's current position as a waypoint '''
	NavDevice.storage.waypoints.append(NavDevice.storage.position)

@NavDevice.on('saveWayPoint')
def saveWayPoint('saveWayPoint',path):
	''' Save waypoints to a file in json format '''
	f = open(path,'w')
	for waypoint in NavDevice.storage.waypoints:
		json.dump(waypoint.__dict__,f)
		f.write('\n')
	NavDevice.storage.waypoints = []

@NavDevice.on(loadWayPoint)
def loadWayPoint_callback('loadWayPoint', path):
	''' Load waypoints from a json file '''
	f = open(path,'r')
	json_data = f.read()
	data_list = json_data.split('\n')
	for s in data_list[0:-1]:
		data = json.loads(s)
		pos = GPSPosition(data['lat'],data['lon'],data['mode'])
		NavDevice.storage.waypoints.append(pos)

@NavDevice.on('clearWayPoint')
def clearWayPoint_callback('clearWayPoint', data):
	''' Clear all waypoints '''
	NavDevice.storage.waypoints = []

@NavDevice.on('autonomousMode')
def autonomousMode_callback('autonomousMode', flag):
	''' Enable autonomous navigation mode '''
	NavDevice.storage.autonomous_mode = flag

NavDevice.run()
