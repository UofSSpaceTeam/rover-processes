from pyvesc import SetRPM, SetCurrent, SetCurrentBrake
import pyvesc
from math import expm1 # e**x - 1  for rpm/current curves
from math import exp
from robocluster import Device

from roverutil import getnetwork

RPM_TO_ERPM = 12*19 # 12 poles, 19:1 gearbox

# Limits for Electronic RPM.
# Note this is not the RPM of the wheel, but the
# speed at which the motor is commutated.
RPMdiv = 4
DEADZONE = 0.1
MAX_RPM = 40000/RPMdiv
MIN_RPM = 300
MAX_CURRENT = 6
MIN_CURRENT = 0.1
CURVE_VAL = 17

WHEEL_RADIUS = 0.26 # meters
ROVER_WIDTH = 1.16 # meters

# Modify the direction of the wheels to account for backwards wiring.
DirectionConstants = {
    'wheelRF':1,
    'wheelRM':1,
    'wheelRB':1,
    'wheelLF':1,
    'wheelLM':1,
    'wheelLB':1,
}

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

#gear is the number that MAX_RPM is divided by (higher gear = slower MAX_RPM
gear = 4
#gearIndex keeps track of which gear in gearList is current gear
gearIndex = 0

def GearShift(Lbumpdown, RbumpDown gear, gearIndex):
    gearList = [4,2,1]
    LbumpState = JoystickDevice.storage.get_button(4)
    RbumpState = JoystickDevice.storage.get_button(5)
    if LbumpDown == False:
        if LbumpState == True:
            #print("button " + str(ButtonNum) + " is down")
            if gearIndex > 0:
                gearIndex -= 1
            else:
                pass
            LbumpState = True
        else:
            pass
    else:
        pass
    if LbumpDown == True:
        if LbumpState == False:
            #print("button " + str(ButtonNum) + " is up")
            LbumpDown = False
        else:
            pass
    else:
        pass

    return LbumpState, RbumpState, gear

DriveDevice = Device('DriveSystem', 'rover', network=getnetwork())

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
            await DriveDevice.publish('wheelLF', {'SetRPM':DirectionConstants['wheelLF']*int(speed)})
            await DriveDevice.publish('wheelLM', {'SetRPM':DirectionConstants['wheelLM']*int(speed)})
            await DriveDevice.publish('wheelLB', {'SetRPM':DirectionConstants['wheelLB']*int(speed)})
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
            await DriveDevice.publish("wheelRF", {'SetRPM':DirectionConstants['wheelRF']*int(speed)})
            await DriveDevice.publish("wheelRM", {'SetRPM':DirectionConstants['wheelRM']*int(speed)})
            await DriveDevice.publish("wheelRB", {'SetRPM':DirectionConstants['wheelRB']*int(speed)})
    elif DriveDevice.storage.drive_mode == "current" and not DriveDevice.storage.right_brake:
            current = austin_current_curve(y_axis)
            #if -MIN_CURRENT < current < MIN_CURRENT:
            #   current = 0
            await DriveDevice.publish("wheelRF", {'SetCurrent':current})
            await DriveDevice.publish("wheelRM", {'SetCurrent':current})
            await DriveDevice.publish("wheelRB", {'SetCurrent':current})

@DriveDevice.on('*/Ltrigger')
async def Ltrigger_callback(Ltrigger, trigger):
    """ Handles left wheel braking (requires current mode)"""
    if 0 < trigger <= 1 and DriveDevice.storage.drive_mode == "current":
            DriveDevice.storage.left_brake = True
            await DriveDevice.publish("wheelLF", {'SetCurrent':max_current})
            await DriveDevice.publish("wheelLM", {'SetCurrent':max_current})
            await DriveDevice.publish("wheelLB", {'SetCurrent':max_current})
    else:
            DriveDevice.storage.left_brake = False

@DriveDevice.on('*/Rtrigger')
async def Rtrigger_callback(Rtrigger, trigger):
    """ Handles right wheel braking (requires current mode)"""
    if 0 < trigger <= 1 and DriveDevice.storage.drive_mode == "current":
            DriveDevice.storage.right_brake = True
            await DriveDevice.publish("wheelRF", {'SetCurrent':max_current})
            await DriveDevice.publish("wheelRM", {'SetCurrent':max_current})
            await DriveDevice.publish("wheelRB", {'SetCurrent':max_current})
    else:
            DriveDevice.storage.right_brake = False


#### Drive API #####
async def setLeftWheelSpeed(rpm):
    rpm = rpm*RPM_TO_ERPM
    rpm = min(rpm, MAX_RPM)
    print(rpm)
    await DriveDevice.publish("wheelLF", {'SetRPM':DirectionConstants['wheelLF']*rpm})
    await DriveDevice.publish("wheelLM", {'SetRPM':DirectionConstants['wheelLM']*rpm})
    await DriveDevice.publish("wheelLB", {'SetRPM':DirectionConstants['wheelLB']*rpm})

async def setRightWheelSpeed(rpm):
    rpm = rpm*RPM_TO_ERPM
    rpm = min(rpm, MAX_RPM)
    print(rpm)
    await DriveDevice.publish("wheelRF", {'SetRPM':DirectionConstants['wheelRF']*rpm})
    await DriveDevice.publish("wheelRM", {'SetRPM':DirectionConstants['wheelRM']*rpm})
    await DriveDevice.publish("wheelRB", {'SetRPM':DirectionConstants['wheelRB']*rpm})

@DriveDevice.on('Stop')
async def DriveStop_callback(event, data):
    await setLeftWheelSpeed(0)
    await setRightWheelSpeed(0)

@DriveDevice.on('DriveForward')
async def DriveForward_callback(DriveForward, speed):
    rpm = speed*60/WHEEL_RADIUS
    await setLeftWheelSpeed(rpm)
    await setRightWheelSpeed(rpm)

@DriveDevice.on('DriveBackward')
async def DriveBackward_callback(DriveBackward, speed):
    rpm = -speed*60/WHEEL_RADIUS
    await setLeftWheelSpeed(rpm)
    await setRightWheelSpeed(rpm)

@DriveDevice.on('RotateRight')
async def DriveRotateRight_callback(DriveRotateRight, speed):
    rpm = speed*60/WHEEL_RADIUS
    await setLeftWheelSpeed(rpm)
    await setRightWheelSpeed(-rpm)

@DriveDevice.on('RotateLeft')
async def DriveRotateLeft_callback(DriveRotateLeft, speed):
    rpm = speed*60/WHEEL_RADIUS
    await setLeftWheelSpeed(-rpm)
    await setRightWheelSpeed(rpm)

DriveDevice.start()
DriveDevice.wait()
