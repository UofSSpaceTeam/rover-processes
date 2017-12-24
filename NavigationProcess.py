# from .GPSProcess import GPSPosition, LOOP_PERIOD
from .GPSdriver import GPSPosition, LOOP_PERIOD
   # ^ Change .GPSProcess? to GPSdriver? Or no?
from math import asin, atan2, cos, pi, radians, sin, sqrt, degrees, atan
import time
#from statistics import mean
from .differential_drive_lib import diff_drive_fk, inverse_kinematics_drive
import json
import math
from .lidarmap import LidarMap

## TODO:
# Replace all instances of self.
# Figure out what's going on with self.position & GPSPosition

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
Navdevice.storage.loop_delay = 100

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

for msg in ["LidarDataMessage", "CompassDataMessage",
			"targetGPS", "singlePointGPS", "GPSVelocity",
			"updateLeftWheelRPM", "updateRightWheelRPM",
			"AccelerometerMessage", "buttonA_down", "buttonB_down",
            "wayPoint", "saveWayPoint","loadWayPoint","clearWayPoint",
			"autonomousMode", "lidarScanFinish"
			]:
	# TODO: Is this the proper way to subscribe in asyncio?
	await Navdevice.subscribe(msg)

# TODO: This was originally a loop(self) function, wasn't 100% sure how to define it using asyncio
@Navdevice.every(Navdevice.storage.loop_delay) # TODO: Not sure what the loop period is supposed to be?
async def every():
	# TODO: If this is the loop period, can this line be an arg for this coro instead of its own sleep Function?
	await Navdevice.sleep(Navdevice.storage.loop_delay / 1000.0)

	if Navdevice.storage.state == "waiting":
		Navdevice.wait_state()
	elif Navdevice.storage.state == "driving":
		Navdevice.drive_state()
	elif Navdevice.storage.state == "manual":
		pass # don't do anything in manual control mode
	else:
		.state = "waiting"
	Navdevice.update_wheel_velocity()
Navdevice.run()

async def drive_state():
	''' Function for handling drive state '''
	if  Navdevice.storage.position is not None:
		# TODO: Not sure where position.distance is initialized. Could it mean GPSPosition, instead of position??
		distance =  Navdevice.storage.position.distance(Navdevice.storage.target)
		# TODO: Not sure where position.bearing is initialized. Could it mean GPSPosition, instead of position??
		bearing = self.position.bearing(Navdevice.storage.target)
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
			#do a lidar scan
			await Navdevice.publish("StartLidarScan",StartLidarScan(1))
			while not Navdevice.storage.lidar_scan_finish:
				await Navdevice.sleep(0.5)
			Navdevice.storage.lidar_scan_finish = False
			# TODO: Unsure where these 2 angles & distance variables are initialized
			lidarMap = LidarMap(self.angles, self.distance)
			if Navdevice.storage.position is not None:
				# TODO: See above function. Unsure where these position.distance, etc are defined
				distance = self.position.distance(Navdevice.storage.waypoints[0])
				bearing = self.position.bearing(Navdevice.storage.waypoints[0])
				if distance > LIDAR_MAX_RANGE:
					distance = LIDAR_MAX_RANGE
				bearing_snap = lidarMap.angle_snap(bearing)
				if lidarMap.distance(bearing_snap) < distance:
					#find a waypoint to avoid the obstacle and set it as target
					angle, distance = lidarMap.find_opening(bearing_snap)
					real_bearing = (heading+ angle) % 360
					# TODO: now gpsPosition is defined as a method of position?? it must mean GPSPosition
					Navdevice.storage.target = self.position.gpsPosition(real_bearing, distance)
				else:
					Navdevice.storage.target = self.waypoints[0]
					Navdevice.storage.waypoints = self.waypoints[1:]
					Navdevice.storage.state = "driving"
	else:
		await Navdevice.publish("DriveStop", 0)

async def update_wheel_velocity():
	Navdevice.storage.right_speed = Navdevice.storage.right_rpm*WHEEL_RADIUS/2
	Navdevice.storage.left_speed = Navdevice.storage.left_rpm*WHEEL_RADIUS/2

async def pos_g_h_filter_vel(z, x0, dx, g, dt=1):
	x_est = x0
	#prediction step
	x_pred = x_est + atan((dx*dt)/GPSPosition.RADIUS)

	# update step
	residual = z - x_pred
	x_est  = x_pred + g * residual
	return x_est

async def pos_g_h_filter_wheel(z, x0, dx, g, dt=1):
	x_est = x0
	#prediction step
	x_pred = x_est + atan((dx)/GPSPosition.RADIUS)

	# update step
	residual = z - x_pred
	x_est  = x_pred + g * residual
	return x_est

async def g_h_filter(z, x0, dx, g, dt=1):
	x_est = x0
	#prediction step
	x_pred = x_est + dt*dx

	# update step
	residual = z - x_pred
	x_est  = x_pred + g * residual
	return x_est

@Navdevice.on('LidarDataMessage')
async def LidarDataMessage_callback('LidarDataMessage', lidarmsg):
	''' LidarDataMessage contains:
		distance (centimeters): The lidar unit fires a laser
			beam directly forwards. When it hits an object,
			the length of this beam is the distance.
		angle (degrees): The angle at which the distance
			measurement was taken.
		tilt (degrees): virtical angle the distance was measured at.
	'''
	Navdevice.storage.lidar_distance[int(lidarmsg.angle / LIDAR_ANGLE_UNIT)] = lidarmsg.distance
	Navdevice.storage.lidar_scan_finish = lidarmsg.finished
	# TODO: Are we still scrapping the logging of any message?
	#self.log("Dist: {} Angle: {} Tilt {}".format(lidarmsg.distance,
	#	lidarmsg.angle/100, lidarmsg.tilt))

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
		# TODO: Are these meant to be GPSPosition.?
		lat = (self.position.lat + pos.lat)/(Navdevice.storage.pos_samples)
		lon = (Navdevice.storage.pos_samples*self.position.lon + pos.lon)/(Navdevice.storage.pos_samples+1)
		Navdevice.storage.pos_samples += 1
		# self.position = GPSPosition(lat, lon) #TODO: ????
		#self.log("{},{}".format(degrees(self.position.lat), degrees(self.position.lon)), "DEBUG")
		await Navdevice.publish("RoverPosition", [degrees(self.position.lat), degrees(self.position.lon)])
		return
	if Navdevice.storage.position is not None:
		k = 0.0 # determens which to trust more; velocity(0), or wheels (1)
		pos_pred_lat_vel = Navdevice.pos_g_h_filter_vel(pos.lat,
				self.position.lat, Navdevice.storage.velocity[0], 0.1, LOOP_PERIOD)
		pos_pred_lon_vel = Navdevice.pos_g_h_filter_vel(pos.lon,
				self.position.lon, Navdevice.storage.velocity[1], 0.1, LOOP_PERIOD)

		fk_pred = diff_drive_fk(0,0, ROVER_WIDTH, Navdevice.storage.heading, Navdevice.storage.left_speed, Navdevice.storage.right_speed, LOOP_PERIOD)
		pos_pred_lat_wheel = Navdevice.pos_g_h_filter_wheel(pos.lon, self.position.lat, fk_pred[0], 0.3, LOOP_PERIOD)
		pos_pred_lon_wheel = Navdevice.pos_g_h_filter_wheel(pos.lon, self.position.lon, fk_pred[1], 0.3, LOOP_PERIOD)
		pos_pred_lat = pos_pred_lat_vel*(1-k) + pos_pred_lat_wheel*k
		pos_pred_lon = pos_pred_lon_vel*(1-k) + pos_pred_lon_wheel*k
		#self.log("{},{}".format(degrees(pos_pred_lat), degrees(pos_pred_lon)), "INFO")
		Navdevice.storage.position_last = Navdevice.storage.position
		# TODO: Does this confirm that self.position = GPSPosition(...,...)?
		Navdevice.storage.position = GPSPosition(pos_pred_lat, pos_pred_lon)
		await Navdevice.publish("RoverPosition", [degrees(pos_pred_lat), degrees(pos_pred_lon)])
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
	if Navdevice.storage.state == "waiting":
		Navdevice.storage.velocity[0] = (Navdevice.storage.vel_samples*Navdevice.storage.velocity[0] + vel[0])/(Navdevice.storage.vel_samples)
		Navdevice.storage.velocity[1] = (Navdevice.storage.vel_samples*Navdevice.storage.velocity[1] + vel[1])/(Navdevice.storage.vel_samples+1)
		Navdevice.storage.vel_samples += 1
	else:
		k = 0.1 #constant determining which to trust more; acceleration(0) or wheels(1)
		v_acc_x = Navdevice.g_h_filter(vel[0], Navdevice.storage.velocity[0], Navdevice.storage.accel[0], 0.2, LOOP_PERIOD)
		v_acc_y = Navdevice.g_h_filter(vel[1], Navdevice.storage.velocity[1], Navdevice.storage.accel[1], 0.2, LOOP_PERIOD)

		v_wheel_x = Navdevice.g_h_filter(vel[0], Navdevice.storage.velocity[0], 0.4, LOOP_PERIOD)
		v_wheel_y = Navdevice.g_h_filter(vel[1], Navdevice.storage.velocity[1], 0.4, LOOP_PERIOD)

		Navdevice.storage.velocity[0] = v_acc_x*(1-k) + v_wheel_x*k

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

## old roverberrypy NavigationProcess.py code
'''
class NavigationProcess(RoverProcess):
	 Aggregates data from GPS, Magnetometer, LIDAR, etc,
	 and makes driving decisions based on the rover's
	 surroundings.

	def setup(self, args):
		self.position = None
		self.position_last = None

		self.heading = 0
		self.heading_last = 0

		self.velocity = [0,0] # m/s, north, east
		self.accel = [0,0] #m/s^2, north, east

		self.bearing_error = 5 # degrees
		self._rotating = False

		self.autonomous_mode = True
		self.state = "manual" #can be "waiting" "driving" or "manual"

		# number of samples in our running average
		self.pos_samples = 0
		self.vel_samples = 0
		self.heading_samples = 0

		self.last_compassmessage = 0

		self.target = None
		self.target_reached_distance = 3  # metres
		self.target_maximum_distance = 10 # metres

		# Delay for loop in miliseconds
		self.loop_delay = 100

		# TODO: Why is this 2000, I found it in DriveProcess.py as min_rpm?
		self.motor_rpm = 2000
		self.right_rpm = 0 # wheel RPMs
		self.left_rpm = 0 # wheel RPMs
		self.right_speed = 0 # velocity of right wheel from axel
		self.left_speed = 0 # velocity of left wheel from axel

		self.starting_calibration_gps = [[],[]] # list of GPS positions that ar averaged
		self.starting_calibration_heading = []

		self.waypoints = []

		self.lidar_angles = [i * LIDAR_ANGLE_UNIT for i in range(0, LIDAR_POINTS)]
		self.lidar_distance = [LIDAR_MAX_RANGE for i in range(0, LIDAR_POINTS)]
		self.lidar_scan_finish = False

		for msg in ["LidarDataMessage", "CompassDataMessage",
					"targetGPS", "singlePointGPS", "GPSVelocity",
					"updateLeftWheelRPM", "updateRightWheelRPM",
					"AccelerometerMessage", "buttonA_down", "buttonB_down",
                    "wayPoint", "saveWayPoint","loadWayPoint","clearWayPoint",
					"autonomousMode", "lidarScanFinish"
					]:
			self.subscribe(msg)

	def loop(self):
		time.sleep(self.loop_delay / 1000.0)

		if self.state == "waiting":
			self.wait_state()
		elif self.state == "driving":
			self.drive_state()
		elif self.state == "manual":
			pass # don't do anything in manual control mode
		else:
			self.log("Navigation in invalid state, reverting to wating...", "WARNING")
			self.state = "waiting"

		self.update_wheel_velocity()

	def drive_state(self):
		'' Function for handling drive state''
		if self.position is not None:
			distance = self.position.distance(self.target)
			bearing = self.position.bearing(self.target)
			difference = bearing - self.heading
			if abs(difference) > self.bearing_error:
				if difference < 0:
					self.publish("DriveRotateLeft", MIN_WHEEL_RPM)
					self.left_rpm = -MIN_WHEEL_RPM
					self.right_rpm = MIN_WHEEL_RPM
				else:
					self.publish("DriveRotateRight", MIN_WHEEL_RPM)
					self.left_rpm = MIN_WHEEL_RPM
					self.right_rpm = -MIN_WHEEL_RPM
				return # back to loop
			elif distance > self.target_reached_distance:
				self.publish("DriveForward", MIN_WHEEL_RPM)
				self.left_rpm = MIN_WHEEL_RPM
				self.right_rpm = MIN_WHEEL_RPM
				return #back to loop
			else:
				self.target = None
				self.publish("TargetReached")
				self.state = "waiting"

	def wait_state(self):
		'' Function for handling waiting state''
		if self.autonomous_mode:
			if len(self.waypoints) == 0:
				self.publish("DriveStop", 0)
			else:
				#do a lidar scan
				self.publish("StartLidarScan",StartLidarScan(1))
				while not self.lidar_scan_finish:
					time.sleep(0.5)
				self.lidar_scan_finish = False
				lidarMap = LidarMap(self.angles, self.distance)
				if self.position is not None:
					distance = self.position.distance(self.waypoints[0])
					bearing = self.position.bearing(self.waypoints[0])
					if distance > LIDAR_MAX_RANGE:
						distance = LIDAR_MAX_RANGE
					bearing_snap = lidarMap.angle_snap(bearing)
					if lidarMap.distance(bearing_snap) < distance:
						#find a waypoint to avoid the obstacle and set it as target
						angle, distance = lidarMap.find_opening(bearing_snap)
						real_bearing = (heading+ angle) % 360
						self.target = self.position.gpsPosition(real_bearing, distance)
					else:
						self.target = self.waypoints[0]
						self.waypoints = self.waypoints[1:]
						self.state = "driving"
		else:
			self.publish("DriveStop", 0)

	def update_wheel_velocity(self):
		self.right_speed = self.right_rpm*WHEEL_RADIUS/2
		self.left_speed = self.left_rpm*WHEEL_RADIUS/2

	def pos_g_h_filter_vel(self, z, x0, dx, g, dt=1):
		x_est = x0
		#prediction step
		x_pred = x_est + atan((dx*dt)/GPSPosition.RADIUS)

		# update step
		residual = z - x_pred
		x_est  = x_pred + g * residual
		return x_est

	def pos_g_h_filter_wheel(self, z, x0, dx, g, dt=1):
		x_est = x0
		#prediction step
		x_pred = x_est + atan((dx)/GPSPosition.RADIUS)

		# update step
		residual = z - x_pred
		x_est  = x_pred + g * residual
		return x_est

	def g_h_filter(self, z, x0, dx, g, dt=1):
		x_est = x0
		#prediction step
		x_pred = x_est + dt*dx

		# update step
		residual = z - x_pred
		x_est  = x_pred + g * residual
		return x_est

	def on_LidarDataMessage(self, lidarmsg):
		''  LidarDataMessage contains:
			distance (centimeters): The lidar unit fires a laser
				beam directly forwards. When it hits an object,
				the length of this beam is the distance.
			angle (degrees): The angle at which the distance
				measurement was taken.
			tilt (degrees): virtical angle the distance was measured at.
		''
		self.lidar_distance[int(lidarmsg.angle / LIDAR_ANGLE_UNIT)] = lidarmsg.distance
		self.lidar_scan_finish = lidarmsg.finished
		self.log("Dist: {} Angle: {} Tilt {}".format(lidarmsg.distance,
			lidarmsg.angle/100, lidarmsg.tilt))

	def on_CompassDataMessage(self, msg):
		''  CompassDataMessage contains:
			heading (degrees): Relative to north, the angle of
				rotation on the axis normal to the earth's surface.
			pitch (degrees):
			roll (degrees):
		''
		if self.heading is not None:
			self.heading_last = self.heading
			if self.state == "waiting":
				# rolling average
				self.heading = (self.heading_samples*self.heading + msg.heading)/(self.heading_samples+1)
				self.heading_samples += 1
			else:
				tmp = time.time()
				d_t = tmp-self.last_compassmessage
				self.last_compassmessage = tmp
				self.heading = self.g_h_filter(msg.heading, self.heading,
						(self.right_speed-self.left_speed)/ROVER_WIDTH, 0.8, d_t)
				self.publish("RoverHeading", self.heading)
			# self.log("heading: {}".format(self.heading))

	def on_targetGPS(self, pos):
		''Targets a new GPS coordinate'
		target = GPSPosition(radians(pos[0]), radians(pos[1]))
		self.publish("TargetReached", False)
		if target.distance(self.position) <= self.maximum_target_distance:
			if len(self.waypoints) > 1:
				self.waypoints.append(target)
				self.target = waypoints[0]
			else:
				self.target = target
			if self.state != "manual":
				self.state = "driving"

	def on_singlePointGPS(self, pos):
		'Updates GPS position''
		#std_dev 1.663596084712623e-05, 2.1743680968892167e-05
		# self.log("{},{}".format(degrees(pos.lat), degrees(pos.lon)))
		if self.state == "waiting" and self.position is not None:
			lat = (self.position.lat + pos.lat)/(self.pos_samples)
			lon = (self.pos_samples*self.position.lon + pos.lon)/(self.pos_samples+1)
			self.pos_samples += 1
			# self.position = GPSPosition(lat, lon)
			self.log("{},{}".format(degrees(self.position.lat), degrees(self.position.lon)), "DEBUG")
			self.publish("RoverPosition", [degrees(self.position.lat), degrees(self.position.lon)])
			return
		if self.position is not None:
			k = 0.0 # determens which to trust more; velocity(0), or wheels (1)
			pos_pred_lat_vel = self.pos_g_h_filter_vel(pos.lat,
					self.position.lat, self.velocity[0], 0.1, LOOP_PERIOD)
			pos_pred_lon_vel = self.pos_g_h_filter_vel(pos.lon,
					self.position.lon, self.velocity[1], 0.1, LOOP_PERIOD)

			fk_pred = diff_drive_fk(0,0, ROVER_WIDTH, self.heading, self.left_speed, self.right_speed, LOOP_PERIOD)
			pos_pred_lat_wheel = self.pos_g_h_filter_wheel(pos.lon, self.position.lat, fk_pred[0], 0.3, LOOP_PERIOD)
			pos_pred_lon_wheel = self.pos_g_h_filter_wheel(pos.lon, self.position.lon, fk_pred[1], 0.3, LOOP_PERIOD)
			pos_pred_lat = pos_pred_lat_vel*(1-k) + pos_pred_lat_wheel*k
			pos_pred_lon = pos_pred_lon_vel*(1-k) + pos_pred_lon_wheel*k
			self.log("{},{}".format(degrees(pos_pred_lat), degrees(pos_pred_lon)), "INFO")
			self.position_last = self.position
			self.position = GPSPosition(pos_pred_lat, pos_pred_lon)
			self.publish("RoverPosition", [degrees(pos_pred_lat), degrees(pos_pred_lon)])
		else:
			if len(self.starting_calibration_gps[0]) < CALIBRATION_SAMPLES:
				self.starting_calibration_gps[0].append(pos.lat)
				self.starting_calibration_gps[1].append(pos.lon)
			else:
				# Done averaging
				self.starting_calibration_gps[0].append(pos.lat)
				self.starting_calibration_gps[1].append(pos.lon)
				self.position = GPSPosition(mean(self.starting_calibration_gps[0]), mean(self.starting_calibration_gps[1]))
				self.log("{},{}".format(degrees(pos.lat), degrees(pos.lon)), "INFO")

	def on_GPSVelocity(self, vel):
		''Updates velocity from GPS unit.''
		# std_dev 0.04679680341613995, 0.035958365746391524
		# self.log("{},{}".format(vel[0], vel[1]))
		if self.state == "waiting":
			self.velocity[0] = (self.vel_samples*self.velocity[0] + vel[0])/(self.vel_samples)
			self.velocity[1] = (self.vel_samples*self.velocity[1] + vel[1])/(self.vel_samples+1)
			self.vel_samples += 1
		else:
			k = 0.1 #constant determining which to trust more; acceleration(0) or wheels(1)
			v_acc_x = self.g_h_filter(vel[0], self.velocity[0], self.accel[0], 0.2, LOOP_PERIOD)
			v_acc_y = self.g_h_filter(vel[1], self.velocity[1], self.accel[1], 0.2, LOOP_PERIOD)

			v_wheel_x = self.g_h_filter(vel[0], self.velocity[0], 0.4, LOOP_PERIOD)
			v_wheel_y = self.g_h_filter(vel[1], self.velocity[1], 0.4, LOOP_PERIOD)

			self.velocity[0] = v_acc_x*(1-k) + v_wheel_x*k

	def on_updateLeftWheelRPM(self, rpm):
		'' Drive process can manually overide wheel rpm''
		self.left_rpm = rpm

	def on_updateRightWheelRPM(self, rpm):
		'' Drive process can manually overide wheel rpm''
		self.right_rpm = rpm

	def on_AccelerometerMessage(self, accel):
		''Update acceleration.''
		#mean when stationary: 0.07603603603603604, 0.6156756756756757
		k = 0.2 # trust factor in our acceleration readings
		stationary_accel = (0.07603603603603604, 0.6156756756756757)
		self.accel[0] = k*(accel.x - stationary_accel[0])
		self.accel[1] = k*(accel.y - stationary_accel[1])
		# self.log("{},{}".format(self.accel[0], self.accel[1]))

	def on_ButtonA_down(self, data):
		'' Go into the "waiting" state.''
		self.state = "waiting"

	def on_ButtonB_down(sel, data):
		'' Go into manual control mode.''
		self.state = "manual"

	def on_ButtonY_down(self, pos):
		''Add the rover's current position as a waypoint.''
		self.waypoints.append(self.position)

	def on_saveWayPoint(self,path):
		''Save waypoints to a file in json format.''
		f = open(path,'w')
		for waypoint in self.waypoints:
			json.dump(waypoint.__dict__,f)
			f.write('\n')
		self.waypoints = []

	def on_loadWayPoint(self, path):
		''Load waypoints from a json file.''
		f = open(path,'r')
		json_data = f.read()
		data_list = json_data.split('\n')
		for s in data_list[0:-1]:
			data = json.loads(s)
			pos = GPSPosition(data['lat'],data['lon'],data['mode'])
			self.waypoints.append(pos)

	def on_clearWayPoint(self,data):
		''Clear all waypoints.''
		self.waypoints = []

	def on_autonomousMode(self,flag):
		''Enable autonomous navigation mode.''
		self.autonomous_mode = flag
