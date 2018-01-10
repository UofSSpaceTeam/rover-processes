from .GPSdriver import GPSPosition, LOOP_PERIOD
from math import asin, atan2, cos, pi, radians, sin, sqrt, degrees, atan
import time
#from statistics import mean
from .differential_drive_lib import diff_drive_fk, inverse_kinematics_drive
import json
import math
from .lidarmap import LidarMap

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


Navdevice = Device('Navdevice', 'demo-device')

# Initialize setup variables
Navdevice.storage.position = None
Navdevice.storage.position_last = None

Navdevice.storage.heading = 0
Navdevice.storage.heading_last = 0

Navdevice.storage.velocity = [0,0] # m/s, north, east
Navdevice.storage.accel = [0,0] # m/s^2, north, east

Navdevice.storage.bearing_error = 5 # degrees
Navdevice.storage._rotating = False

Navdevice.storage.autonomous_mode = True
Navdevice.storage.state = "manual" # can be "waiting", "driving", or "manual"

# number of samples in our running average
Navdevice.storage.pos_samples = 0
Navdevice.storage.vel_samples = 0
Navdevice.storage.heading_samples = 0

Navdevice.storage.last_compassmessage = 0

Navdevice.storage.target = None
Navdevice.storage.target_reached_distance = 3  # metres
Navdevice.storage.target_maximum_distance = 10 # metres

# Delay for loop in miliseconds
Navdevice.storage.loop_delay = 0.1

# TODO: Why is this 2000, I found it in DriveProcess.py as min_rpm?
Navdevice.storage.motor_rpm = 2000
Navdevice.storage.right_rpm = 0 # wheel RPMs
Navdevice.storage.left_rpm = 0 # wheel RPMs
Navdevice.storage.right_speed = 0 # velocity of right wheel from axel
Navdevice.storage.left_speed = 0 # velocity of left wheel from axel

Navdevice.storage.starting_calibration_gps = [[],[]] # list of GPS positions that ar averaged
Navdevice.storage.starting_calibration_heading = []

Navdevice.storage.waypoints = []

Navdevice.storage.lidar_angles = [i * LIDAR_ANGLE_UNIT for i in range(0, LIDAR_POINTS)]
Navdevice.storage.lidar_distance = [LIDAR_MAX_RANGE for i in range(0, LIDAR_POINTS)]
Navdevice.storage.lidar_scan_finish = False

@Navdevice.every(Navdevice.storage.loop_delay) # TODO: Not sure what the loop period is supposed to be?
async def every():
	if Navdevice.storage.state == "waiting":
		Navdevice.wait_state()
	elif Navdevice.storage.state == "driving":
		Navdevice.drive_state()
	elif Navdevice.storage.state == "manual":
		pass # don't do anything in manual control mode
	else:
		.state = "waiting"
	Navdevice.update_wheel_velocity()

async def drive_state():
	''' Function for handling drive state '''
	if  Navdevice.storage.position is not None:
		distance =  Navdevice.storage.position.distance(Navdevice.storage.target)
		bearing = Navdevice.storage.position.bearing(Navdevice.storage.target)
		difference = bearing - Navdevice.storage.heading
		if abs(difference) > Navdevice.storage.bearing_error:
			if difference < 0:
				await Navdevice.publish("DriveRotateLeft", MIN_WHEEL_RPM)
				Navdevice.storage.left_rpm = -MIN_WHEEL_RPM
				Navdevice.storage.right_rpm = MIN_WHEEL_RPM
			else:
				await Navdevice.publish("DriveRotateRight", MIN_WHEEL_RPM)
				Navdevice.storage.left_rpm = MIN_WHEEL_RPM
				Navdevice.storage.right_rpm = -MIN_WHEEL_RPM
			return # back to loop
		elif distance > Navdevice.storage.target_reached_distance:
			await Navdevice.publish("DriveForward", MIN_WHEEL_RPM)
			Navdevice.storage.left_rpm = MIN_WHEEL_RPM
			Navdevice.storage.right_rpm = MIN_WHEEL_RPM
			return #back to loop
		else:
			Navdevice.storage.target = None
			await Navdevice.publish("TargetReached")
			Navdevice.storage.state = "waiting"

async def wait_state():
	''' Function for handling waiting state '''
	if Navdevice.storage.autonomous_mode:
		if len(Navdevice.storage.waypoints) == 0:
			await Navdevice.publish("DriveStop", 0)
	else:
		await Navdevice.publish("DriveStop", 0)

def update_wheel_velocity():
	Navdevice.storage.right_speed = Navdevice.storage.right_rpm*WHEEL_RADIUS/2
	Navdevice.storage.left_speed = Navdevice.storage.left_rpm*WHEEL_RADIUS/2

@Navdevice.on('CompassDataMessage')
async def CompassDataMessage_callback('CompassDataMessage', msg):
	''' CompassDataMessage contains:
		heading (degrees): Relative to north, the angle of
			rotation on the axis normal to the earth's surface.
		pitch (degrees):
		roll (degrees):
	'''
	if Navdevice.storage.heading is not None:
		Navdevice.storage.heading_last = Navdevice.storage.heading
		if Navdevice.storage.state == "waiting":
			# rolling average
			Navdevice.storage.heading = (Navdevice.storage.heading_samples*Navdevice.storage.heading + msg.heading)/(Navdevice.storage.heading_samples+1)
			Navdevice.storage.heading_samples += 1
		else:
			# TODO: replace time.time() ?
			tmp = time.time()
			d_t = tmp-Navdevice.storage.last_compassmessage
			Navdevice.storage.last_compassmessage = tmp
			Navdevice.storage.heading = Navdevice.g_h_filter(msg.heading, Navdevice.storage.heading,
					(Navdevice.storage.right_speed-Navdevice.storage.left_speed)/ROVER_WIDTH, 0.8, d_t)
			await Navdevice.publish("RoverHeading", Navdevice.storage.heading)
		# self.log("heading: {}".format(self.heading))

@Navdevice.on('targetGPS')
async def targetGPS_callback('targetGPS', pos):
	''' Targets a new GPS coordinate '''
	target = GPSPosition(radians(pos[0]), radians(pos[1]))
	await Navdevice.publish("TargetReached", False)
	if target.distance(Navdevice.storage.position) <= Navdevice.storage.maximum_target_distance:
		if len(Navdevice.storage.waypoints) > 1:
			Navdevice.storage.waypoints.append(target)
			Navdevice.storage.target = waypoints[0]
		else:
			Navdevice.storage.target = target
		if Navdevice.storage.state != "manual":
			Navdevice.storage.state = "driving"

@Navdevice.on('singlePointGPS')
async def singlePointGPS_callback('singlePointGPS', pos):
	''' Updates GPS position '''
	#std_dev 1.663596084712623e-05, 2.1743680968892167e-05
	# self.log("{},{}".format(degrees(pos.lat), degrees(pos.lon)))
	if Navdevice.storage.state == "waiting" and Navdevice.storage.position is not None:
		lat = (Navdevice.storage.position.lat + pos.lat)/(Navdevice.storage.pos_samples)
		lon = (Navdevice.storage.pos_samples*Navdevice.storage.position.lon + pos.lon)/(Navdevice.storage.pos_samples+1)
		Navdevice.storage.pos_samples += 1
		await Navdevice.publish("RoverPosition", [degrees(Navdevice.storage.position.lat), degrees(Navdevice.storage.position.lon)])
		return
	if Navdevice.storage.position is not None:
		Navdevice.storage.position_last = Navdevice.storage.position
		Navdevice.storage.position = pos
		await Navdevice.publish("RoverPosition", [degrees(pos.lat), degrees(pos.lon)])
	else:
		if len(Navdevice.storage.starting_calibration_gps[0]) < CALIBRATION_SAMPLES:
			Navdevice.storage.starting_calibration_gps[0].append(pos.lat)
			Navdevice.storage.starting_calibration_gps[1].append(pos.lon)
		else:
			# Done averaging
			Navdevice.storage.starting_calibration_gps[0].append(pos.lat)
			Navdevice.storage.starting_calibration_gps[1].append(pos.lon)
			Navdevice.storage.position = GPSPosition(mean(Navdevice.storage.starting_calibration_gps[0]),
				mean(Navdevice.storage.starting_calibration_gps[1]))
			#self.log("{},{}".format(degrees(pos.lat), degrees(pos.lon)), "INFO")

@Navdevice.on('GPSVelocity')
async def GPSVelocity_callback('GPSVelocity', vel):
	''' Updates velocity from GPS unit '''
	# std_dev 0.04679680341613995, 0.035958365746391524
	# self.log("{},{}".format(vel[0], vel[1]))
	Navdevice.storage.velocity = vel


@Navdevice.on('ButtonA_down')
async def ButtonA_down_callback('ButtonA_down', data):
	''' Go into the "waiting" state '''
	Navdevice.storage.state = "waiting"

@Navdevice.on('ButtonB_down')
async def ButtonB_down_callback('ButtonB_down', data):
	''' Go into manual control mode '''
	Navdevice.storage.state = "manual"

@Navdevice.on('ButtonY_down')
async def ButtonY_down_callback('ButtonY_down', pos):
	''' Add the rover's current position as a waypoint '''
	Navdevice.storage.waypoints.append(Navdevice.storage.position)

@Navdevice.on('saveWayPoint')
async def saveWayPoint('saveWayPoint',path):
	''' Save waypoints to a file in json format '''
	f = open(path,'w')
	for waypoint in Navdevice.storage.waypoints:
		json.dump(waypoint.__dict__,f)
		f.write('\n')
	Navdevice.storage.waypoints = []

@Navdevice.on(loadWayPoint)
async def loadWayPoint_callback('loadWayPoint', path):
	''' Load waypoints from a json file '''
	f = open(path,'r')
	json_data = f.read()
	data_list = json_data.split('\n')
	for s in data_list[0:-1]:
		data = json.loads(s)
		pos = GPSPosition(data['lat'],data['lon'],data['mode'])
		Navdevice.storage.waypoints.append(pos)

@Navdevice.on('clearWayPoint')
async def clearWayPoint_callback('clearWayPoint', data):
	''' Clear all waypoints '''
	Navdevice.storage.waypoints = []

@Navdevice.on('autonomousMode')
async def autonomousMode_callback('autonomousMode', flag):
	''' Enable autonomous navigation mode '''
	Navdevice.storage.autonomous_mode = flag

Navdevice.run()
