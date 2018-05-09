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

Arm = Device('Arm','rover')

Arm.storage.base_direction = None
Arm.storage.joints_pos = None
Arm.storage.speeds = None
Arm.storage.command = None
Arm.storage.section_lengths = None #Not actually from 'self'
Arm.storage.joint_limits = None
Arm.storage.max_angular_velocity = None
Arm.storage.config = None #May not need this one in storage.
Arm.storage.controller = None #Ditto
Arm.storage.mode = None #Ditto
Arm.storage.devices = None #dytto
Arm.storage.joint_offsets = None # D to I to T T O

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
	
class ArmProcess(RoverProcess):

	def setup(self, args):
		for key in ["joystick1", "joystick2", "triggerR", "triggerL", "dpad", "buttonB_down","buttonA_down","buttonY_down", "buttonA_up","buttonY_up"]:
			self.subscribe(key)
		for key in device_keys:
			self.subscribe(key)
		self.base_direction = None
		self.joints_pos = Joints(0, 0, pi/4, 0, 0, 0)
		self.speeds = Joints(0,0,0,0,0,0)
		self.command = [0,0,0,0,0,0]
		section_lengths = Sections(
				upper_arm=0.35,
				forearm=0.42,
				end_effector=0.1)
		joint_limits = Joints(
				# in radians
				base=None,
				shoulder=Limits(-0.09, 0.721),
				elbow=Limits(1.392, 1.699),
				wrist_pitch=None,
				wrist_roll=None,
				gripper=None)
		max_angular_velocity = Joints(
				base=0.6,
				shoulder=0.4,
				elbow=0.4,
				wrist_pitch=0.4,
				wrist_roll=0.8,
				gripper=0.8)
		self.config = Config(section_lengths, joint_limits, max_angular_velocity)
		self.controller = Controller(self.config)
		self.mode = ManualControl()
		self.devices = {}
		# joint_offsets are values in degrees to 'zero' the encoder angle
		self.joint_offsets = {"d_armShoulder":-332.544, "d_armElbow":-221.505+90}
		# self.joint_offsets = {"d_armShoulder":0, "d_armElbow":0}


	def simulate_positions(self):
		''' Updates the positions by calculating new values for testing.'''
		new_joints = list(self.joints_pos)
		for i in range(len(self.speeds)):
			if new_joints[i] is not None:
				new_joints[i] = self.joints_pos[i] + self.speeds[i] * dt
		return Joints(*new_joints)

	def poll_encoder(self, device):
		''' Polls each VESC for its encoder position.'''
		with serial.Serial(self.devices[device], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
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
				self.log("Failed to read rotor position {}".format(device), "ERROR")
		return None

	def get_positions(self):
		''' Returns an updated Joints object with the current arm positions'''
		new_joints = list(self.joints_pos)
		for i, device in enumerate(["d_armShoulder", "d_armElbow", "d_armWristPitch"]):
			if device in self.devices:
				reading = self.poll_encoder(device)
				if reading is not None:
					if device == "d_armShoulder" and reading < 180:
						# The shoulder joint's magnet happens to be orientated
						# that the encoder flips from 360 to 0 partway through
						# the rotation.
						reading += 360
					reading += self.joint_offsets[device]
					new_joints[i+1] = round(math.radians(reading), 3) #Convert to radians
				else:
					self.log("Could not read joint position {}".format(device), "WARNING")
		return Joints(*new_joints) def loop(self):
		self.joints_pos = self.get_positions()
		self.log("command: {}".format(self.command), "DEBUG")
		self.controller.user_command(self.mode, *self.command)
		self.speeds = self.controller.update_duties(self.joints_pos)
		#publish speeds/duty cycles here
		self.log("joints_pos: {}".format(self.joints_pos), "DEBUG")
		self.log("speeds: {}".format(self.speeds), "DEBUG")
		self.send_duties()
		time.sleep(dt)

	def send_duties(self):
		''' Tell each motor controller to turn on motors'''
		if "d_armBase" in self.devices:
			with serial.Serial(self.devices["d_armBase"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
				ser.write(pyvesc.encode(SetDutyCycle(int(100000*self.speeds[0]))))
		if "d_armShoulder" in self.devices:
			with serial.Serial(self.devices["d_armShoulder"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
				ser.write(pyvesc.encode(SetDutyCycle(int(100000*self.speeds[1]))))
		if "d_armElbow" in self.devices:
			with serial.Serial(self.devices["d_armElbow"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
				ser.write(pyvesc.encode(SetDutyCycle(int(100000*self.speeds[2]))))
		if "d_armWristRot" in self.devices:
			with serial.Serial(self.devices["d_armWristRot"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
				ser.write(pyvesc.encode(SetDutyCycle(int(100000*self.speeds[4]))))
		if "d_armGripperOpen" in self.devices:
			with serial.Serial(self.devices["d_armGripperOpen"], baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
				ser.write(pyvesc.encode(SetDutyCycle(int(100000*self.speeds[5]))))

	@Arm.on('*/joystick1') 
	async def on_joystick1(event, data): ''' Shoulder joint, and radius control.'''
		#self.log("joystick1:{}".format(data), "DEBUG")
		y_axis = data[1]
		if isinstance(event.mode, ManualControl):
			y_axis *= -1*shoulder_max_speed
			if y_axis > shoulder_min_speed or y_axis < -shoulder_min_speed:
				armShoulderSpeed = y_axis
			else:
				armShoulderSpeed = 0
			event.command[1] = armShoulderSpeed
		elif isinstance(event.mode, PlanarControl):
			y_axis = (y_axis * radius_max_speed)
			if y_axis > radius_min_speed or y_axis < -radius_min_speed:
				radius_speed = y_axis
			else:
				radius_speed = 0
			event.command[0] = radius_speed

	@Arm.on('*/joystick2')
	async def on_joystick2(event, data):
		''' Elbow joints and z/height control'''
		#self.log("joystick2:{}".format(data), "DEBUG")
		y_axis = data[1]
		if isinstance(event.mode, ManualControl):
			y_axis *= elbow_max_speed
			if y_axis > elbow_min_speed or y_axis < -elbow_min_speed:
				armY_ElbowSpeed = y_axis
			else:
				armY_ElbowSpeed = 0
			event.command[2] = armY_ElbowSpeed
		elif isinstance(event.mode, PlanarControl):
			y_axis = (y_axis * height_max_speed)
			if y_axis > height_min_speed or y_axis < -height_min_speed:
				height_speed = y_axis
			else:
				height_speed = 0
			event.command[1] = height_speed
	@Arm.on('*/dpad')
	async def on_dpad(event, data):
		x_axis = data[0]
		y_axis = data[1]
		event.command[3] = y_axis*wrist_pitch_speed
		event.command[4] = x_axis*gripper_rotation_speed

	@Arm.on('*/triggerR')
	async def on_triggerR(event, trigger):
		''' Base rotation right'''
		#self.log("triggerR:{}".format(trigger), "DEBUG")
		trigger = (trigger + 1)/2
		armBaseSpeed = trigger * base_max_speed/2
		if event.base_direction is "left" or event.base_direction is None:
			if -base_min_speed <armBaseSpeed < base_min_speed:
				armBaseSpeed = 0
				event.base_direction = None
			else:
				event.base_direction = "left"
			if isinstance(event.mode, ManualControl):
				event.command[0] = armBaseSpeed
			elif isinstance(event.mode, PlanarControl):
				event.command[2] = armBaseSpeed


	@Arm.on(triggerL)
	async def on_triggerL(event, trigger):
		''' Base rotation left'''
		#self.log("triggerL:{}".format(trigger), "DEBUG")
		trigger = -1*(trigger + 1)/2
		armBaseSpeed = trigger * base_max_speed/2
		if event.base_direction is "right" or event.base_direction is None:
			if -base_min_speed <armBaseSpeed < base_min_speed:
				armBaseSpeed = 0
				event.base_direction = None
			else:
				event.base_direction = "right"
			if isinstance(event.mode, ManualControl):
				event.command[0] = armBaseSpeed
			elif isinstance(event.mode, PlanarControl):
				event.command[2] = armBaseSpeed
	@Arm.on('*/buttonB_down')
	async def on_buttonB_down(event, data):
		if isinstance(event.mode, ManualControl):
			event.mode = PlanarControl()
			event.log("PlanarControl")
		else:
			event.mode = ManualControl()
			event.log("ManualControl")
	@Arm.on('buttonA_down')
	async def on_buttonA_down(event, data):
		event.log("gripper close:{}".format(data), "DEBUG")
		event.command[5] = gripper_open_speed

	@Arm.on('buttonA_up')
	async def on_buttonA_up(event,data):
		event.log("gripper close stop:{}".format(data), "DEBUG")
		event.command[5] = 0

	@Arm.on('buttonY_up')
	async def on_buttonY_up(event,data):
		event.log("gripper open stop:{}".format(data), "DEBUG")
		event.command[5] = 0

	@Arm.on('buttonY_down')
	async def on_buttonY_down(event, data):
		event.log("gripper open:{}".format(data), "DEBUG")
		event.command[5] = -gripper_open_speed

	def messageTrigger(event, message):
		if message.key in device_keys:
			event.log("Received device: {} at {}".format(message.key, message.data), "DEBUG")
			self.devices[message.key] = message.data
			with serial.Serial(message.data, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
				# Turn on encoder readings for this VESC
				ser.write(pyvesc.encode(
					SetRotorPositionMode(
						SetRotorPositionMode.DISP_POS_MODE_ENCODER )))







