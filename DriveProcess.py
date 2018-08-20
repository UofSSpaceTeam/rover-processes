"""
DriveProcess
============
Handles anything to do with driving the rover. Provides manual control,
and a remote API for other processes such as Autopilot to move the rover.
The DriveProcess uses events from an xbox controller for manual control.
Currently manual control is set up like a skid steer where two joysticks
control each side of the wheels. We primary send commands to the motor controllers
to control the RPM of the wheels, but a mode for setting the current is
also available. The current control mode has not been used much, so make
sure you test reasonable values are being sent out before you hook up the
motors to the controllers.
"""
from pyvesc import SetRPM, SetCurrent, SetCurrentBrake
import pyvesc
from math import expm1 # e**x - 1  for rpm/current curves
from math import exp
from robocluster import Device

import config

log = config.getLogger()

RPM_TO_ERPM = 12*19 # 12 poles, 19:1 gearbox

CONTROLLER_NUM = config.drive_controller

# Limits for Electronic RPM.
# Note this is not the RPM of the wheel, but the
# speed at which the motor is commutated.
MAX_RPM = 40000
MIN_RPM = 300
MAX_RPM_CHANGE = MAX_RPM/2
Autonav_MAX_RPM = 10000
MAX_CURRENT = 1
MIN_CURRENT = 0.1
CURVE_VAL = 17
MIN_DIVISOR = 1
MAX_DIVISOR = 4

# Joystick deadzone
DEADZONE = 0.1

WHEEL_RADIUS = 0.26 # meters
ROVER_WIDTH = 1.16 # meters

# Modify the direction of the wheels to account for backwards wiring.
DirectionConstants = {
    'wheelRF':1,
    'wheelRM':1,
    'wheelRB':1,
    'wheelLF':1,
    'wheelLM':-1,
    'wheelLB':1,
}

def austins_curve(f):
    a = ((CURVE_VAL**abs(f)) - 1)/(CURVE_VAL - 1)
    if f > 0:
        return a
    else:
        return -a

DriveDevice = Device('DriveSystem', 'rover', network=config.network)

# Initialize setup variables
DriveDevice.storage.right_brake = False
DriveDevice.storage.left_brake = False
DriveDevice.storage.drive_mode = "rpm" # "rpm" or "current"
DriveDevice.storage.api_enabled = False
DriveDevice.storage.left_rpm = 0
DriveDevice.storage.right_rpm = 0
DriveDevice.storage.divisor = 4

@DriveDevice.on('*/controller{}/joystick1'.format(CONTROLLER_NUM))
async def joystick1_callback(joystick1, data):
    """ Handles the left wheels for manual control.
            A joystick1 message contains:
            [x axis (float -1:1), y axis (float -1:1)]
    """
    if DriveDevice.storage.api_enabled:
        return
    y_axis = data[1]
    if y_axis is None:
            return
    if DriveDevice.storage.drive_mode == "rpm":
            speed = austins_curve(y_axis)*MAX_RPM/DriveDevice.storage.divisor
            if -DEADZONE < y_axis < DEADZONE: # DEADZONE
                    speed = 0
            await DriveDevice.publish('wheelLF', {'SetRPM':DirectionConstants['wheelLF']*int(speed)})
            await DriveDevice.publish('wheelLM', {'SetRPM':DirectionConstants['wheelLM']*int(speed)})
            await DriveDevice.publish('wheelLB', {'SetRPM':DirectionConstants['wheelLB']*int(speed)})
    elif DriveDevice.storage.drive_mode == "current" and not DriveDevice.storage.left_brake:
            current = austins_curve(y_axis)*MAX_CURRENT*100
            await DriveDevice.publish("wheelLF", {'SetCurrent':current})
            await DriveDevice.publish("wheelLM", {'SetCurrent':current})
            await DriveDevice.publish("wheelLB", {'SetCurrent':current})

@DriveDevice.on('*/controller{}/joystick2'.format(CONTROLLER_NUM))
async def joystick2_callback(joystick2, data):
    """ Handles the right wheels for manual control.
            A joystick1 message contains:
            [x axis (float -1:1), y axis (float -1:1)]
    """
    if DriveDevice.storage.api_enabled:
        return
    y_axis = data[1]
    if y_axis is None:
            return
    if DriveDevice.storage.drive_mode == "rpm":
            speed = austins_curve(y_axis)*MAX_RPM/DriveDevice.storage.divisor
            if -DEADZONE < y_axis < DEADZONE: # DEADZONE
                    speed = 0
            await DriveDevice.publish("wheelRF", {'SetRPM':DirectionConstants['wheelRF']*int(speed)})
            await DriveDevice.publish("wheelRM", {'SetRPM':DirectionConstants['wheelRM']*int(speed)})
            await DriveDevice.publish("wheelRB", {'SetRPM':DirectionConstants['wheelRB']*int(speed)})
    elif DriveDevice.storage.drive_mode == "current" and not DriveDevice.storage.right_brake:
            current = austins_curve(y_axis)*MAX_CURRENT*100
            #if -MIN_CURRENT < current < MIN_CURRENT:
            #   current = 0
            await DriveDevice.publish("wheelRF", {'SetCurrent':current})
            await DriveDevice.publish("wheelRM", {'SetCurrent':current})
            await DriveDevice.publish("wheelRB", {'SetCurrent':current})

@DriveDevice.on('*/controller{}/trigger'.format(CONTROLLER_NUM))
async def trigger_callback(Ltrigger, trigger):
    """ Handles left wheel braking (requires current mode)"""
    if 0 < trigger <= 1 and DriveDevice.storage.drive_mode == "current":
            DriveDevice.storage.left_brake = True
            await DriveDevice.publish("wheelLF", {'SetCurrent':max_current})
            await DriveDevice.publish("wheelLM", {'SetCurrent':max_current})
            await DriveDevice.publish("wheelLB", {'SetCurrent':max_current})
            await DriveDevice.publish("wheelRF", {'SetCurrent':max_current})
            await DriveDevice.publish("wheelRM", {'SetCurrent':max_current})
            await DriveDevice.publish("wheelRB", {'SetCurrent':max_current})
    else:
            DriveDevice.storage.left_brake = False

@DriveDevice.on('*/controller{}/bumperR_down'.format(CONTROLLER_NUM))
def up_shift(event, data):
    """
    Increase the max speed of the drive system.
    """
    d = DriveDevice.storage.divisor/2
    d = max(MIN_DIVISOR, d)
    DriveDevice.storage.divisor = d


@DriveDevice.on('*/controller{}/bumperL_down'.format(CONTROLLER_NUM))
def down_shift(event, data):
    """
    Decrease the max speed of the drive system.
    """
    d = DriveDevice.storage.divisor*2
    d = min(MAX_DIVISOR, d)
    DriveDevice.storage.divisor = d


#### Drive API #####

async def setLeftWheelSpeed(rpm):
    """
    Set the speed of the left wheels.
    Args:
        rpm (int): The ERPM to send to the wheels.
    """
    left_rpm = DriveDevice.storage.left_rpm
    rpm = rpm*RPM_TO_ERPM
    # limit the accelration for smoother movement.
    d_rpm = min(abs(rpm-left_rpm), max(abs(left_rpm)/Autonav_MAX_RPM, MIN_RPM/Autonav_MAX_RPM)*MAX_RPM_CHANGE)
    if rpm > 0:
        if rpm > left_rpm: # speeding up
            rpm = min(left_rpm+d_rpm, Autonav_MAX_RPM)
        else: # slowing down
            rpm = min(left_rpm-d_rpm, Autonav_MAX_RPM)
    elif rpm < 0:
        if rpm > left_rpm: #slowing down
            rpm = max(left_rpm+d_rpm, -Autonav_MAX_RPM)
        else:
            rpm = max(left_rpm-d_rpm, -Autonav_MAX_RPM)
    else:
        rpm = 0
    log.debug('Left RPM: {}'.format(rpm))
    if DriveDevice.storage.api_enabled:
        DriveDevice.storage.left_rpm = rpm
        await DriveDevice.publish("wheelLF", {'SetRPM':DirectionConstants['wheelLF']*int(rpm)})
        await DriveDevice.publish("wheelLM", {'SetRPM':DirectionConstants['wheelLM']*int(rpm)})
        await DriveDevice.publish("wheelLB", {'SetRPM':DirectionConstants['wheelLB']*int(rpm)})

async def setRightWheelSpeed(rpm):
    """
    Set the speed of the right wheels.
    Args:
        rpm (int): The ERPM to send to the wheels.
    """
    right_rpm = DriveDevice.storage.right_rpm
    rpm = rpm*RPM_TO_ERPM
    # limit the accelration for smoother movement.
    d_rpm = min(abs(rpm-right_rpm), max(abs(right_rpm)/Autonav_MAX_RPM, MIN_RPM/Autonav_MAX_RPM)*MAX_RPM_CHANGE)
    if rpm > 0:
        if rpm > right_rpm:
            rpm = min(right_rpm+d_rpm, Autonav_MAX_RPM)
        else:
            rpm = min(right_rpm-d_rpm, Autonav_MAX_RPM)
    elif rpm < 0:
        if rpm > right_rpm:
            rpm = max(right_rpm+d_rpm, -Autonav_MAX_RPM)
        else:
            rpm = max(right_rpm-d_rpm, -Autonav_MAX_RPM)
    else:
        rpm = 0
    log.debug('Right RPM: {}'.format(rpm))
    if DriveDevice.storage.api_enabled:
        DriveDevice.storage.right_rpm = rpm
        await DriveDevice.publish("wheelRF", {'SetRPM':DirectionConstants['wheelRF']*int(rpm)})
        await DriveDevice.publish("wheelRM", {'SetRPM':DirectionConstants['wheelRM']*int(rpm)})
        await DriveDevice.publish("wheelRB", {'SetRPM':DirectionConstants['wheelRB']*int(rpm)})

@DriveDevice.on('*/Autopilot')
def enable_api(event, data):
    """
    Enable or disable the drive API.
    Args:
        data (boolean): Set the state of the drive API.
    """
    DriveDevice.storage.api_enabled = data

@DriveDevice.on('Stop')
async def DriveStop_callback(event, data):
    """
    Stop the rover by setting the wheel speed to 0.
    """
    await setLeftWheelSpeed(0)
    await setRightWheelSpeed(0)

@DriveDevice.on('DriveForward')
async def DriveForward_callback(DriveForward, speed):
    """
    Drive the rover forward at a given speed.
    Args:
        speed (float): Speed in m/s to drive at.
    """
    rpm = speed*60/WHEEL_RADIUS
    if DriveDevice.storage.left_rpm != DriveDevice.storage.right_rpm:
        rpm = 0
    await setLeftWheelSpeed(rpm)
    await setRightWheelSpeed(rpm)

@DriveDevice.on('DriveBackward')
async def DriveBackward_callback(DriveBackward, speed):
    """
    Drive the rover backwards at a given speed.
    Args:
        speed (float): Speed in m/s to drive at.
    """
    rpm = -speed*60/WHEEL_RADIUS
    await setLeftWheelSpeed(rpm)
    await setRightWheelSpeed(rpm)

@DriveDevice.on('RotateRight')
async def DriveRotateRight_callback(DriveRotateRight, speed):
    """
    While stationary, rotate on the spot to the right.
    Args:
        speed (float): Speed to rotate at in m/s... Should probably
                       update this to be a more sensible unit.
    """
    rpm = speed*60/WHEEL_RADIUS
    await setLeftWheelSpeed(rpm)
    await setRightWheelSpeed(-rpm)

@DriveDevice.on('RotateLeft')
async def DriveRotateLeft_callback(DriveRotateLeft, speed):
    """
    While stationary, rotate on the spot to the left.
    Args:
        speed (float): Speed to rotate at in m/s... Should probably
                       update this to be a more sensible unit.
    """
    rpm = speed*60/WHEEL_RADIUS
    await setLeftWheelSpeed(-rpm)
    await setRightWheelSpeed(rpm)

@DriveDevice.on('TurnRight')
async def DriveTurnRight_callback(DriveTurnRight, speed):
    """
    Turn right while still driving forward.
    """
    rpm = speed*60/WHEEL_RADIUS
    await setLeftWheelSpeed(rpm)
    await setRightWheelSpeed(rpm*0.5)

@DriveDevice.on('TurnLeft')
async def DriveTurnLeft_callback(DriveTurnLeft, speed):
    """
    Turn right while still driving forward.
    """
    rpm = speed*60/WHEEL_RADIUS
    await setLeftWheelSpeed(rpm*0.5)
    await setRightWheelSpeed(rpm)

DriveDevice.start()
DriveDevice.wait()
