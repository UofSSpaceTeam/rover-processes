# Copyright 2016 University of Saskatchewan Space Design Team Licensed under the
# Educational Community License, Version 2.0 (the "License"); you may
# not use this file except in compliance with the License. You may
# obtain a copy of the License at
#
# https://opensource.org/licenses/ecl2.php
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an "AS IS"
# BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
# or implied. See the License for the specific language governing
# permissions and limitations under the License.

from .RoverProcess import RoverProcess
import pyvesc
from pyvesc import SetDutyCycle, GetRotorPosition, SetRotorPositionMode
from roverprocess.arm17.arm import Joints, Controller, Config, ManualControl,Sections,Limits,PlanarControl
from math import pi
import math
import serial

# Any libraries you need can be imported here. You almost always need time!
import time
from robocluster import Device


base_max_speed = 3
base_min_speed = 0.4
shoulder_max_speed = 2
shoulder_min_speed = 0.1
elbow_max_speed = 2
elbow_min_speed = 0.1
wrist_pitch_speed = 2
gripper_rotation_speed = 2
gripper_open_speed = 1

radius_max_speed = 2
radius_min_speed = 0.2
height_max_speed = 2
height_min_speed = 0.2

device_keys = ["d_armBase", "d_armShoulder", "d_armElbow", "d_armWristRot", "d_armGripperOpen"]

dt = 0.01
BAUDRATE = 115200
SERIAL_TIMEOUT = 0.02

ArmDevice = Device('ArmDevice','rover')

ArmDevice.storage.base_direction = None
ArmDevice.storage.joints_pos = None
ArmDevice.storage.speeds = None
ArmDevice.storage.command = None
ArmDevice.storage.section_lengths = None #Not actually from self 
ArmDevice.storage.joint_limits = None # Also was not from self
ArmDevice.storage.max_angular_velocity = None #Again, not from self
ArmDevice.storage.config = None #May not need this one in storage.
ArmDevice.storage.controller = None #Ditto
ArmDevice.storage.mode = None #Ditto
ArmDevice.storage.devices = None #dytto
ArmDevice.storage.joint_offsets = None # D to I to T T O

'''
 setup()
	:param: arm_storage - a device's storage, in this case, that of the arm.

	:synopsis: Takes in a device's and initalizes all contained parameters
		   to the values from the old class.
'''
def setup(arm_storage):
	arm_storage.base_direction = None
	arm_storage.joints_pos = Joints(0,0,pi/4,0,0,0)
	arm_storage.speeds = Joints(0,0,0,0,0,0)
	arm_storage.command = [0,0,0,0,0,0]
	arm_storage.section_lengths = Sections(
				upper_arm = 0.35,
				forearm = 0.42,
				end_effector = 0.1)
	arm_storage.joint_limits = Joints(
				base = None,
				shoulder = Limits(-0.09, 0.721),
				elbow = Limits(1.392,1.699),
				wrist_pitch = None,
				wrist_roll = None,
				gripper = None)
	arm_storage.max_angular_velocity = Joints(
					base = 0.6,
					shoulder = 0.4,
					elbow = 0.4,
					wrist_pitch = 0.4,
					wrist_roll = 0.8,
					gripper = 0.8)
		arm_storage.config = Config(
					arm_storage.section_lengths,
					arm_storage.joint_limits,
					arm_stoage.max_angular_velocity)
		arm_storage.controller = Controller(arm_storage.config)
		arm_stoarge.mode = ManualControl()
		arm_storage.devices = {}
		#joint_offesets are values in degrees to 'zero' the encoder angle
		arm_storage.joint_offsets = {'d_armShoulder':-332.544,
					     'd_armElbow':-221.505+90}
	

def simulate_positions():
	''' Updates the positions by calculating new values for testing.'''
	new_joints = list(ArmDevice.storage.joints_pos)
	for i in range(len(ArmDevice.storage.speeds)):
		if new_joints[i] is not None:
			new_joints[i] = ArmDevice.storage.joints_pos[i] + 
					ArmDevice.storage.speeds[i] * dt
	return Joints(*new_joints)

def poll_encoder(device):
	''' Polls each VESC for its encoder position.'''
	with serial.Serial(ArmDevice.storeage.devices[device], 
		baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:

		ser.write(pyvesc.encode_request(GetRotorPosition))
		# while ser.in_waiting < 9:
		# 	# Wait for response. TODO: Maybe don't wait forever...
		# 	pass
		buffer = ser.read(10) # size of a RotorPosition message
		try:
			(response, consumed) = pyvesc.decode(buffer)
			if response.__class__ == GetRotorPosition:
				return response.rotor_pos
		except:
			print("Failed to read rotor position {}".format(device), "ERROR")
	return None

def get_positions():
	''' Returns an updated Joints object with the current arm positions'''
	new_joints = list(ArmDevice.storage.joints_pos)
	for i, device in enumerate(["d_armShoulder", "d_armElbow", "d_armWristPitch"]):
		if device in ArmDevice.storage.devices:
			reading = ArmDevice.storage.poll_encoder(device)
			if reading is not None:
				if device == "d_armShoulder" and reading < 180:
					# The shoulder joint's magnet happens to be orientated
					# that the encoder flips from 360 to 0 partway through
					# the rotation.
					reading += 360
				reading += ArmDevice.storage.joint_offsets[device]
				new_joints[i+1] = round(math.radians(reading), 3) #Convert to radians
			else:
				print("Could not read joint position {}".format(device), "WARNING")
	return Joints(*new_joints) 

# I don't think we use this
def loop():
	ArmDevice.storage.joints_pos = get_positions()
	print("command: {}".format(ArmDevice.storage.command), "DEBUG")
	ArmDevice.storage.controller.user_command(ArmDevice.storage.mode, *ArmDevice.storage.command) #Keep an eye on that pointer.
	ArmDevice.storage.speeds = ArmDevice.storage.controller.update_duties(ArmDevice.storage.joints_pos)
	
	#publish speeds/duty cycles here
	print("joints_pos: {}".format(ArmDevice.storage.joints_pos), "DEBUG")
	print("speeds: {}".format(ArmDevice.storage.speeds), "DEBUG")
	
	send_duties() #This may have not work, used to have a self
	time.sleep(dt)

def send_duties():
	''' Tell each motor controller to turn on motors'''
	if "d_armBase" in ArmDevice.storage.devices:
		with serial.Serial(ArmDevice.storage.devices["d_armBase"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
			ser.write(pyvesc.encode(SetDutyCycle(int(100000*ArmDevice.storage.speeds[0]))))
	if "d_armShoulder" in ArmDevice.storage.devices:
		with serial.Serial(ArmDevice.storage.devices["d_armShoulder"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
			ser.write(pyvesc.encode(SetDutyCycle(int(100000*ArmDevice.storage.speeds[1]))))
	if "d_armElbow" in ArmDevice.storage.devices:
		with serial.Serial(ArmDevice.storage.devices["d_armElbow"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
			ser.write(pyvesc.encode(SetDutyCycle(int(100000*ArmDevice.storage.speeds[2]))))
	if "d_armWristRot" in ArmDevice.storage.devices:
		with serial.Serial(ArmDevice.storage.devices["d_armWristRot"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
			ser.write(pyvesc.encode(SetDutyCycle(int(100000*ArmDevice.storage.speeds[4]))))
	if "d_armGripperOpen" in ArmDevice.storage.devices:
		with serial.Serial(ArmDevice.storage.devices["d_armGripperOpen"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
			ser.write(pyvesc.encode(SetDutyCycle(int(100000*ArmDevice.storage.speeds[5]))))

@ArmDevice.on('*/joystick1') 
async def on_joystick1(event, data): ''' Shoulder joint, and radius control.'''
	#print("joystick1:{}".format(data), "DEBUG")
	y_axis = data[1]
	if isinstance(ArmDevice.storage.mode, ManualControl):
		y_axis *= -1*shoulder_max_speed
		if y_axis > shoulder_min_speed or y_axis < -shoulder_min_speed:
			armShoulderSpeed = y_axis
		else:
			armShoulderSpeed = 0
		ArmDevice.storage.command[1] = armShoulderSpeed
	elif isinstance(ArmDevice.storage.mode, PlanarControl):
		y_axis = (y_axis * radius_max_speed)
		if y_axis > radius_min_speed or y_axis < -radius_min_speed:
			radius_speed = y_axis
		else:
			radius_speed = 0
		ArmDevice.storage.command[0] = radius_speed

@ArmDevice.on('*/joystick2')
async def on_joystick2(event, data):
	''' Elbow joints and z/height control'''
	y_axis = data[1]
	if isinstance(ArmDevice.storage.mode, ManualControl):
		y_axis *= elbow_max_speed
		if y_axis > elbow_min_speed or y_axis < -elbow_min_speed:
			armY_ElbowSpeed = y_axis
		else:
			armY_ElbowSpeed = 0
		ArmDevice.storage.command[2] = armY_ElbowSpeed
	elif isinstance(ArmDevice.storage.mode, PlanarControl):
		y_axis = (y_axis * height_max_speed)
		if y_axis > height_min_speed or y_axis < -height_min_speed:
			height_speed = y_axis
		else:
			height_speed = 0
		ArmDevice.storage.command[1] = height_speed

@ArmDevice.on('*/dpad')
async def on_dpad(event, data):
	x_axis = data[0]
	y_axis = data[1]
	ArmDevice.storage.command[3] = y_axis*wrist_pitch_speed
	ArmDevice.storage.command[4] = x_axis*gripper_rotation_speed

@ArmDevice.on('*/triggerR')
async def on_triggerR(event, trigger):
	''' Base rotation right'''
	#print("triggerR:{}".format(trigger), "DEBUG")
	trigger = (trigger + 1)/2
	armBaseSpeed = trigger * base_max_speed/2
	if ArmDevice.storage.base_direction is "left" or ArmDevice.storage.base_direction is None:
		if -base_min_speed <armBaseSpeed < base_min_speed:
			armBaseSpeed = 0
			ArmDevice.storage.base_direction = None
		else:
			ArmDevice.storage.base_direction = "left"
		if isinstance(ArmDevice.storage.mode, ManualControl):
			ArmDevice.storage.command[0] = armBaseSpeed
		elif isinstance(ArmDevice.storage.mode, PlanarControl):
			ArmDevice.storage.command[2] = armBaseSpeed


@ArmDevice.on('*/triggerL')
async def on_triggerL(event, trigger):
	''' Base rotation left'''
	#print("triggerL:{}".format(trigger), "DEBUG")
	trigger = -1*(trigger + 1)/2
	armBaseSpeed = trigger * base_max_speed/2
	if ArmDevice.storage.base_direction is "right" or ArmDevice.storage.base_direction is None:
		if -base_min_speed <armBaseSpeed < base_min_speed:
			armBaseSpeed = 0
			ArmDevice.storage.base_direction = None
		else:
			ArmDevice.storage.base_direction = "right"
		if isinstance(ArmDevice.storage.mode, ManualControl):
			ArmDevice.storage.command[0] = armBaseSpeed
		elif isinstance(ArmDevice.storage.mode, PlanarControl):
			ArmDevice.storage.command[2] = armBaseSpeed

@ArmDevice.on('*/buttonB_down')
async def on_buttonB_down(event, data):
	if isinstance(ArmDevice.storage.mode, ManualControl):
		ArmDevice.storage.mode = PlanarControl()
		print("PlanarControl")
	else:
		ArmDevice.storage.mode = ManualControl()
		print("ManualControl")

@ArmDevice.on('*/buttonA_down')
async def on_buttonA_down(event, data):
	print("gripper close:{}".format(data), "DEBUG")
	ArmDevice.storage.command[5] = gripper_open_speed

@ArmDevice.on('*/buttonA_up')
async def on_buttonA_up(event,data):
	print("gripper close stop:{}".format(data), "DEBUG")
	ArmDevice.storage.command[5] = 0

@ArmDevice.on('*/buttonY_up')
async def on_buttonY_up(event,data):
	print("gripper open stop:{}".format(data), "DEBUG")
	ArmDevice.storage.command[5] = 0

@ArmDevice.on('*/buttonY_down')
async def on_buttonY_down(event, data):
	print("gripper open:{}".format(data), "DEBUG")
	ArmDevice.storage.command[5] = -gripper_open_speed

def messageTrigger(event, message):
	if message.key in device_keys:
		print("Received device: {} at {}".format(message.key, message.data), "DEBUG")
		ArmDevice.storage.devices[message.key] = message.data
		with serial.Serial(message.data, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
			# Turn on encoder readings for this VESC
			ser.write(pyvesc.encode(
				SetRotorPositionMode(
					SetRotorPositionMode.DISP_POS_MODE_ENCODER )))



ArmDevice.start()
ArmDevice.wait()



