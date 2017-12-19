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

from pyvesc import SetRPM, SetCurrent, SetCurrentBrake
import pyvesc
from math import expm1 # e**x - 1  for rpm/current curves
from math import exp

RPM_TO_ERPM = 12*19 # 12 poles, 19:1 gearbox

# Limits for Electronic RPM.
# Note this is not the RPM of the wheel, but the
# speed at which the motor is commutated.

DEADZONE = 0.1
MAX_RPM = 40000
MIN_RPM = 300
MAX_CURRENT = 6
MIN_CURRENT = 0.1
CURVE_VAL = 17

def rpm_curve(f):
	if f > 0:
		rpm = (1/5)*MAX_RPM + (4/5)*MAX_RPM*((expm1(2*f))/(expm1(4)))
	elif f < 0:
		f = -1*f
		rpm = (1/5)*MAX_RPM + (4/5)*MAX_RPM*((expm1(2*f))/(expm1(4)))
		rpm = -1*rpm
	else:
		rpm = 0

	#print(rpm)
	return rpm

def current_curve(f):
	if f > 0:
		current = (1/5)*MAX_CURRENT + (4/5)*MAX_CURRENT*((expm1(2*f))/(expm1(4)))
	elif f < 0:
		f = -1*f
		current = (1/5)*MAX_CURRENT + (4/5)*MAX_CURRENT*((expm1(2*f))/(expm1(4)))
		current = -1*current
	else:
		current = 0
	return current

def austin_rpm_curve(f):

	a = ((CURVE_VAL**abs(f)) - 1)/(CURVE_VAL - 1)
	
	if f > 0:
		#print(a*40000)
		return a*MAX_RPM
	else:
		#print(-a*40000)
		return -a*MAX_RPM

def austin_current_curve(f):
	
	a = ((CURVE_VAL**abs(f)) - 1)/(CURVE_VAL - 1)
	if f > 0:
		return a*MAX_CURRENT*100

	else:
		return -a*MAX_CURRENT*100


Drivedevice = Device('Drivedevice', 'demo-device')

# Initialize setup variables
Drivedevice.storage.right_brake = False
Drivedevice.storage.left_brake = False
Drivedevice.storage.drive_mode = "rpm" # "rpm" or "current"

@Drivedevice.on('joystick1')
async def joystick1_callback(joystick1, data):
        """ Handles the left wheels for manual control.
                A joystick1 message contains:
                [x axis (float -1:1), y axis (float -1:1)]
        """
        y_axis = data[1]
        if y_axis is None:
                return
        if Drivedevice.storage.drive_mode == "rpm":
                speed = austin_rpm_curve(y_axis)
                if -DEADZONE < y_axis < DEADZONE: # DEADZONE
                        speed = 0
                await Drivedevice.publish("wheelLF", SetRPM(int(speed)))
                await Drivedevice.publish("wheelLM", SetRPM(int(-1*speed)))
                await Drivedevice.publish("wheelLB", SetRPM(int(speed)))
                await Drivedevice.publish("updateLeftWheelRPM", speed)
        elif Drivedevice.storage.drive_mode == "current" and not Drivedevice.storage.left_brake:
                current = austin_current_curve(y_axis)
                await Drivedevice.publish("wheelLF", SetCurrent(current))
                await Drivedevice.publish("wheelLM", SetCurrent(current))
                await Drivedevice.publish("wheelLB", SetCurrent(current))

@Drivedevice.on('joystick2')
async def joystick2_callback(joystick2, data)
        """ Handles the right wheels for manual control.
                A joystick1 message contains:
                [x axis (float -1:1), y axis (float -1:1)]
        """
        y_axis = data[1]
        if y_axis is None:
                return
        if Drivedevice.storage.drive_mode == "rpm":
                speed = austin_rpm_curve(y_axis)
                if -DEADZONE < y_axis < DEADZONE: # DEADZONE
                        speed = 0
                await Drivedevice.publish("wheelRF", SetRPM(int(speed)))
                await Drivedevice.publish("wheelRM", SetRPM(int(speed)))
                await Drivedevice.publish("wheelRB", SetRPM(int(-1*speed)))
                await Drivedevice.publish("updateRightWheelRPM", speed)
        elif Drivedevice.storage.drive_mode == "current" and not Drivedevice.storage.right_brake:
                current = austin_current_curve(y_axis)
                #if -MIN_CURRENT < current < MIN_CURRENT:
                #	current = 0
                await Drivedevice.publish("wheelRF", SetCurrent(current))
                await Drivedevice.publish("wheelRM", SetCurrent(current))
                await Drivedevice.publish("wheelRB", SetCurrent(current))
        # Single drive mode not working due to missing axis on Windows
        #if Drivedevice.drive_mode == "single":
        #	speed = rpm_curve(y_axis)
        #	if -DEADZONE < y_axis < DEADZONE: # DEADZONE
        #		speed = 0
        #	Drivedevice.publish("wheelRF", SetRPM(int( 1*(speed + self.mix))))
        #	Drivedevice.publish("wheelRM", SetRPM(int( 1*(speed + self.mix))))
        #	Drivedevice.publish("wheelRB", SetRPM(int(-1*(speed + self.mix))))
        #	Drivedevice.publish("wheelLF", SetRPM(int( 1*(speed - self.mix))))
        #	Drivedevice.publish("wheelLM", SetRPM(int(-1*(speed - self.mix))))
        #	Drivedevice.publish("wheelLB", SetRPM(int( 1*(speed - self.mix))))
        #	#self.log("right: {}".format(speed))

@Drivedevice.on('Ltrigger')
async def Ltrigger_callback(Ltrigger, trigger):
        """ Handles left wheel braking (requires current mode)"""
        if 0 < trigger <= 1 and Drivedevice.storage.drive_mode == "current":
                Drivedevice.storage.left_brake = True
                await Drivedevice.publish("wheel1", SetCurrentBrake(max_current))
                await Drivedevice.publish("wheel2", SetCurrentBrake(max_current))
                await Drivedevice.publish("wheel3", SetCurrentBrake(max_current))
        else:
                Drivedevice.storage.left_brake = False

@Drivedevice.on('Rtrigger')
async def Rtrigger_callback(Rtrigger, trigger):
        """ Handles right wheel braking (requires current mode)"""
        if 0 < trigger <= 1 and Drivedevice.storage.drive_mode == "current":
                Drivedevice.storage.right_brake = True
                await Drivedevice.publish("wheel4", SetCurrentBrake(MAX_CURRENT))
                await Drivedevice.publish("wheel5", SetCurrentBrake(MAX_CURRENT))
                await Drivedevice.publish("wheel6", SetCurrentBrake(MAX_CURRENT))
        else:
                Drivedevice.storage.right_brake = False

@Drivedevice.on('ButtonA_down')
async def ButtonA_down_callback(ButtonA_down, val):
	await Drivedevice.publish("autoDrive")

@Drivedevice.on('ButtonB_down')
async def ButtonB_down_callback(ButtonB_down, val):
        await Drivedevice.publish("manualDrive")

async def _setLeftWheelSpeed(self, rpm):
        rpm = SetRPM(int(rpm))
        await Drivedevice.publish("wheelLF", rpm)
        await Drivedevice.publish("wheelLM", rpm)
        await Drivedevice.publish("wheelLB", rpm)

async def _setRightWheelSpeed(self, rpm):
        rpm = SetRPM(int(rpm))
        await Drivedevice.publish("wheelRF", rpm)
        await Drivedevice.publish("wheelRM", rpm)
        await Drivedevice.publish("wheelRB", rpm)

@Drivedevice.on('DriveStop')
async def DriveStop_callback(DriveStop, data):
        Drivedevice._setLeftWheelSpeed(0)
        Drivedevice._setRightWheelSpeed(0)

@Drivedevice.on('DriveForward')
async def DriveForward_callback(DriveForward, speed):
        Drivedevice._setLeftWheelSpeed(speed*RPM_TO_ERPM)
        Drivedevice._setRightWheelSpeed(speed*RPM_TO_ERPM)

@Drivedevice.on('DriveBackward')
async def DriveBackward_callback(DriveBackward, speed):
        Drivedevice._setLeftWheelSpeed(-speed*RPM_TO_ERPM)
        Drivedevice._setRightWheelSpeed(-speed*RPM_TO_ERPM)

@Drivedevice.on('DriveRotateRight')
async def DriveRotateRight_callback(DriveRotateRight, speed):
        Drivedevice._setLeftWheelSpeed(speed*RPM_TO_ERPM)
        Drivedevice._setRightWheelSpeed(-speed*RPM_TO_ERPM)

@Drivedevice.on('DriveRotateLeft')
async def DriveRotateLeft_callback(DriveRotateLeft, speed):
        Drivedevice._setLeftWheelSpeed(-speed*RPM_TO_ERPM)
        Drivedevice._setRightWheelSpeed(speed*RPM_TO_ERPM)




## Old roveberryPi DriveProcess Code
'''
class DriveProcess(RoverProcess):
	"""Handles driving the rover.

	Takes joystick input from the web ui and
	commands the wheels to move. Uses RPM and current control modes.
	"""

	def setup(self, args):
		""" Initialize drive mode (default=rpm)."""
		self.right_brake = False
		self.left_brake = False
		self.drive_mode = "rpm"
		for key in ["joystick1", "joystick2","triggerL","triggerR", "on_DriveStop",
					"on_DriveForward", "on_DriveBackward",
					"on_DriveRotateRight", "on_DriveRotateLeft", "buttonA_down"]:
			self.subscribe(key)

	def on_joystick1(self, data):
		""" Handles the left wheels for manual control.
			A joystick1 message contains:
			[x axis (float -1:1), y axis (float -1:1)]
		"""
		y_axis = data[1]
		if y_axis is None:
			return
		if self.drive_mode == "rpm":
			self.log("rpm")
			speed = austin_rpm_curve(y_axis)
			if -DEADZONE < y_axis < DEADZONE: # DEADZONE
				speed = 0
			self.publish("wheelLF", SetRPM(int(speed)))
			self.publish("wheelLM", SetRPM(int(-1*speed)))
			self.publish("wheelLB", SetRPM(int(speed)))
			self.publish("updateLeftWheelRPM", speed)
			self.log("left: {}".format(speed))
		elif self.drive_mode == "current" and not self.left_brake:
			current = austin_current_curve(y_axis)
			self.publish("wheelLF", SetCurrent(current))
			self.publish("wheelLM", SetCurrent(current))
			self.publish("wheelLB", SetCurrent(current))
			self.log(current)



	def on_joystick2(self, data):
		""" Handles the right wheels for manual control.
			A joystick1 message contains:
			[x axis (float -1:1), y axis (float -1:1)]
		"""
		y_axis = data[1]
		print(data)
		if y_axis is None:
			return
		if self.drive_mode == "rpm":
			speed = austin_rpm_curve(y_axis)
			if -DEADZONE < y_axis < DEADZONE: # DEADZONE
				speed = 0
			self.publish("wheelRF", SetRPM(int(speed)))
			self.publish("wheelRM", SetRPM(int(speed)))
			self.publish("wheelRB", SetRPM(int(-1*speed)))
			self.publish("updateRightWheelRPM", speed)
			self.log("right: {}".format(speed))
		elif self.drive_mode == "current" and not self.right_brake:
			current = austin_current_curve(y_axis)
			#if -MIN_CURRENT < current < MIN_CURRENT:
			#	current = 0
			self.publish("wheelRF", SetCurrent(current))
			self.publish("wheelRM", SetCurrent(current))
			self.publish("wheelRB", SetCurrent(current))
			self.log(current)
		# Single drive mode not working due to missing axis on Windows
		#if self.drive_mode == "single":
		#	speed = rpm_curve(y_axis)
		#	if -DEADZONE < y_axis < DEADZONE: # DEADZONE
		#		speed = 0
		#	self.publish("wheelRF", SetRPM(int( 1*(speed + self.mix))))
		#	self.publish("wheelRM", SetRPM(int( 1*(speed + self.mix))))
		#	self.publish("wheelRB", SetRPM(int(-1*(speed + self.mix))))
		#	self.publish("wheelLF", SetRPM(int( 1*(speed - self.mix))))
		#	self.publish("wheelLM", SetRPM(int(-1*(speed - self.mix))))
		#	self.publish("wheelLB", SetRPM(int( 1*(speed - self.mix))))
		#	#self.log("right: {}".format(speed))

	def on_Ltrigger(self, trigger):
		""" Handles left wheel braking (requires current mode)"""
		if 0 < trigger <= 1 and self.drive_mode == "current":
			self.left_brake = True
			self.publish("wheel1", SetCurrentBrake(max_current))
			self.publish("wheel2", SetCurrentBrake(max_current))
			self.publish("wheel3", SetCurrentBrake(max_current))
		else:
			self.left_brake = False

	def on_Rtrigger(self, trigger):
		""" Handles right wheel braking (requires current mode)"""
		if 0 < trigger <= 1 and self.drive_mode == "current":
			self.right_brake = True
			self.publish("wheel4", SetCurrentBrake(MAX_CURRENT))
			self.publish("wheel5", SetCurrentBrake(MAX_CURRENT))
			self.publish("wheel6", SetCurrentBrake(MAX_CURRENT))
		else:
			self.right_brake = False

	def on_ButtonA_down(self, val):
		self.publish("autoDrive")

	def on_ButtonB_down(self, val):
		self.publish("manualDrive")

	def _setLeftWheelSpeed(self, rpm):
		rpm = SetRPM(int(rpm))
		self.publish("wheelLF", rpm)
		self.publish("wheelLM", rpm)
		self.publish("wheelLB", rpm)

	def _setRightWheelSpeed(self, rpm):
		rpm = SetRPM(int(rpm))
		self.publish("wheelRF", rpm)
		self.publish("wheelRM", rpm)
		self.publish("wheelRB", rpm)

	def on_DriveStop(self, data):
		self._setLeftWheelSpeed(0)
		self._setRightWheelSpeed(0)

	def on_DriveForward(self, speed):
		self._setLeftWheelSpeed(speed*RPM_TO_ERPM)
		self._setRightWheelSpeed(speed*RPM_TO_ERPM)

	def on_DriveBackward(self, speed):
		self._setLeftWheelSpeed(-speed*RPM_TO_ERPM)
		self._setRightWheelSpeed(-speed*RPM_TO_ERPM)

	def on_DriveRotateRight(self, speed):
		self._setLeftWheelSpeed(speed*RPM_TO_ERPM)
		self._setRightWheelSpeed(-speed*RPM_TO_ERPM)

	def on_DriveRotateLeft(self, speed):
		self._setLeftWheelSpeed(-speed*RPM_TO_ERPM)
		self._setRightWheelSpeed(speed*RPM_TO_ERPM)









