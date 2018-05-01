from pyvesc import SetRPM, SetCurrent, SetCurrentBrake
import pyvesc
from math import expm1 # e**x - 1  for rpm/current curves
from math import exp
from robocluster import Device

RPM_TO_ERPM = 12*19 # 12 poles, 19:1 gearbox

# Limits for Electronic RPM.
# Note this is not the RPM of the wheel, but the
# speed at which the motor is commutated.

DEADZONE = 0.1
MAX_RPM = 40000/2
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


DriveDevice = Device('DriveDevice', 'rover')

# Initialize setup variables
DriveDevice.storage.right_brake = False
DriveDevice.storage.left_brake = False
DriveDevice.storage.drive_mode = "rpm" # "rpm" or "current"

@DriveDevice.on('*/joystick1')
async def joystick1_callback(joystick1, data):
        """ Handles the left wheels for manual control.
                A joystick1 message contains:
                [x axis (float -1:1), y axis (float -1:1)]
        """
        y_axis = data[1]
        if y_axis is None:
                return
        if DriveDevice.storage.drive_mode == "rpm":
                speed = austin_rpm_curve(y_axis)
                if -DEADZONE < y_axis < DEADZONE: # DEADZONE
                        speed = 0
                await DriveDevice.publish('wheelLF', {'SetRPM':int(speed)})
                await DriveDevice.publish('wheelLM', {'SetRPM':int(speed)})
                await DriveDevice.publish('wheelLB', {'SetRPM':int(speed)})
        elif DriveDevice.storage.drive_mode == "current" and not DriveDevice.storage.left_brake:
                current = austin_current_curve(y_axis)
                await DriveDevice.publish("wheelLF", {'SetCurrent':current})
                await DriveDevice.publish("wheelLM", {'SetCurrent':current})
                await DriveDevice.publish("wheelLB", {'SetCurrent':current})

@DriveDevice.on('*/joystick2')
async def joystick2_callback(joystick2, data):
        """ Handles the right wheels for manual control.
                A joystick1 message contains:
                [x axis (float -1:1), y axis (float -1:1)]
        """
        y_axis = data[1]
        if y_axis is None:
                return
        if DriveDevice.storage.drive_mode == "rpm":
                speed = austin_rpm_curve(y_axis)
                if -DEADZONE < y_axis < DEADZONE: # DEADZONE
                        speed = 0
                await DriveDevice.publish("wheelRF", {'SetRPM':int(speed)})
                await DriveDevice.publish("wheelRM", {'SetRPM':int(speed)})
                await DriveDevice.publish("wheelRB", {'SetRPM':int(speed)})
        elif DriveDevice.storage.drive_mode == "current" and not DriveDevice.storage.right_brake:
                current = austin_current_curve(y_axis)
                #if -MIN_CURRENT < current < MIN_CURRENT:
                #   current = 0
                await DriveDevice.publish("wheelRF", {'SetCurrent':current})
                await DriveDevice.publish("wheelRM", {'SetCurrent':current})
                await DriveDevice.publish("wheelRB", {'SetCurrent':current})
        # Single drive mode not working due to missing axis on Windows
        #if DriveDevice.drive_mode == "single":
        #   speed = rpm_curve(y_axis)
        #   if -DEADZONE < y_axis < DEADZONE: # DEADZONE
        #       speed = 0
        #   DriveDevice.publish("wheelRF", SetRPM(int( 1*(speed + self.mix))))
        #   DriveDevice.publish("wheelRM", SetRPM(int( 1*(speed + self.mix))))
        #   DriveDevice.publish("wheelRB", SetRPM(int(-1*(speed + self.mix))))
        #   DriveDevice.publish("wheelLF", SetRPM(int( 1*(speed - self.mix))))
        #   DriveDevice.publish("wheelLM", SetRPM(int(-1*(speed - self.mix))))
        #   DriveDevice.publish("wheelLB", SetRPM(int( 1*(speed - self.mix))))
        #   #self.log("right: {}".format(speed))

@DriveDevice.on('Ltrigger')
async def Ltrigger_callback(Ltrigger, trigger):
        """ Handles left wheel braking (requires current mode)"""
        if 0 < trigger <= 1 and DriveDevice.storage.drive_mode == "current":
                DriveDevice.storage.left_brake = True
                await DriveDevice.publish("wheel1", SetCurrentBrake(max_current))
                await DriveDevice.publish("wheel2", SetCurrentBrake(max_current))
                await DriveDevice.publish("wheel3", SetCurrentBrake(max_current))
        else:
                DriveDevice.storage.left_brake = False

@DriveDevice.on('Rtrigger')
async def Rtrigger_callback(Rtrigger, trigger):
        """ Handles right wheel braking (requires current mode)"""
        if 0 < trigger <= 1 and DriveDevice.storage.drive_mode == "current":
                DriveDevice.storage.right_brake = True
                await DriveDevice.publish("wheel4", SetCurrentBrake(MAX_CURRENT))
                await DriveDevice.publish("wheel5", SetCurrentBrake(MAX_CURRENT))
                await DriveDevice.publish("wheel6", SetCurrentBrake(MAX_CURRENT))
        else:
                DriveDevice.storage.right_brake = False

@DriveDevice.on('ButtonA_down')
async def ButtonA_down_callback(ButtonA_down, val):
    await DriveDevice.publish("autoDrive")

@DriveDevice.on('ButtonB_down')
async def ButtonB_down_callback(ButtonB_down, val):
        await DriveDevice.publish("manualDrive")

async def _setLeftWheelSpeed(rpm):
        rpm = SetRPM(int(rpm))
        await DriveDevice.publish("wheelLF", rpm)
        await DriveDevice.publish("wheelLM", rpm)
        await DriveDevice.publish("wheelLB", rpm)

async def _setRightWheelSpeed(rpm):
        rpm = SetRPM(int(rpm))
        await DriveDevice.publish("wheelRF", rpm)
        await DriveDevice.publish("wheelRM", rpm)
        await DriveDevice.publish("wheelRB", rpm)

@DriveDevice.on('DriveStop')
def DriveStop_callback(DriveStop, data):
        DriveDevice._setLeftWheelSpeed(0)
        DriveDevice._setRightWheelSpeed(0)

@DriveDevice.on('DriveForward')
def DriveForward_callback(DriveForward, speed):
        DriveDevice._setLeftWheelSpeed(speed*RPM_TO_ERPM)
        DriveDevice._setRightWheelSpeed(speed*RPM_TO_ERPM)

@DriveDevice.on('DriveBackward')
def DriveBackward_callback(DriveBackward, speed):
        DriveDevice._setLeftWheelSpeed(-speed*RPM_TO_ERPM)
        DriveDevice._setRightWheelSpeed(-speed*RPM_TO_ERPM)

@DriveDevice.on('DriveRotateRight')
def DriveRotateRight_callback(DriveRotateRight, speed):
        DriveDevice._setLeftWheelSpeed(speed*RPM_TO_ERPM)
        DriveDevice._setRightWheelSpeed(-speed*RPM_TO_ERPM)

@DriveDevice.on('DriveRotateLeft')
def DriveRotateLeft_callback(DriveRotateLeft, speed):
        DriveDevice._setLeftWheelSpeed(-speed*RPM_TO_ERPM)
        DriveDevice._setRightWheelSpeed(speed*RPM_TO_ERPM)

DriveDevice.start()
DriveDevice.wait()
