"""
ArmProcess
==========
Controls the arm on the rover.
Utilizes a library written by Liam Bindle to handle
limits and inverse kinematics: https://github.com/LiamBindle/arm17
For manual control, it works pretty similar to
the DriveProcess.
This process is getting to be a bit of a mess, being
ported from our old framework, and should probably
be rewritten from scratch along with Liam's library.
"""

from libraries.arm17.arm import Joints, Controller, Config, ManualControl,Sections,Limits,PlanarControl
from math import pi
import math

from robocluster import Device
import config
log = config.getLogger()

CONTROLLER_NUMBER = config.arm_controller


base_max_speed = 1
base_min_speed = 0.2
shoulder_max_speed = 1
shoulder_min_speed = 0.2
elbow_max_speed = 1
elbow_min_speed = 0.05
forearm_roll_speed = 1
wrist_pitch_speed = 1
gripper_rotation_speed = 1
gripper_open_speed = 1

radius_max_speed = 1
radius_min_speed = 0.2
height_max_speed = 1
height_min_speed = 0.2

base_erpm = 5000
shoulder_erpm = 5000
elbow_erpm = (shoulder_erpm/4/2400)* (12*176.4)
manual_elbow_erpm = 2000
wrist_pitch_erpm = 2000

dt = 0.1

ArmDevice = Device('ArmDevice','rover', network=config.network)

ArmDevice.storage.joints_pos = None
ArmDevice.storage.speeds = None
ArmDevice.storage.command = None
ArmDevice.storage.section_lengths = None
ArmDevice.storage.joint_limits = None
ArmDevice.storage.max_angular_velocity = None
ArmDevice.storage.config = None
ArmDevice.storage.controller = None
ArmDevice.storage.mode = None
ArmDevice.storage.joint_offsets = None

def setup(arm_storage):
    '''
     setup()
        :param: arm_storage - a device's storage, in this case, that of the arm.

        :synopsis: Takes in a device's storage and initalizes all contained parameters
               to the values from the old class.
    '''
    arm_storage.joints_pos = Joints(0,0,pi/4,0,0,0,0)
    arm_storage.speeds = Joints(0,0,0,0,0,0,0)
    arm_storage.command = [0,0,0,0,0,0,0]
    arm_storage.section_lengths = Sections(
            upper_arm = 0.411,
            forearm = 0.444,
            end_effector = 0.1)
    arm_storage.joint_limits = Joints(
            base = None,
            # shoulder = Limits(-0.155, 0.387),
            # elbow = Limits(0.644,2.108),
            shoulder = None,
            elbow = None,
            forearm_roll = None,
            wrist_pitch = None,
            wrist_roll = None,
            gripper = None)
    arm_storage.max_angular_velocity = Joints(
            base = base_max_speed,
            shoulder = shoulder_max_speed,
            elbow = elbow_max_speed,
            forearm_roll = forearm_roll_speed,
            wrist_pitch = wrist_pitch_speed,
            wrist_roll = gripper_rotation_speed,
            gripper = gripper_open_speed)
    arm_storage.config = Config(
            arm_storage.section_lengths,
            arm_storage.joint_limits,
            arm_storage.max_angular_velocity)
    arm_storage.controller = Controller(arm_storage.config)
    arm_storage.mode = ManualControl()
    arm_storage.devices = {}
    #joint_offesets are values in degrees to 'zero' the encoder angle
    arm_storage.joint_offsets = {
        'armShoulder':-324.29444,
        'armElbow':-283.0957+90,
        'armForearmRot':0,
        'armWristPitch':0,
    }


def simulate_positions():
    ''' Updates the positions by calculating new values for testing.'''
    new_joints = list(ArmDevice.storage.joints_pos)
    for i in range(len(ArmDevice.storage.speeds)):
        if new_joints[i] is not None:
            new_joints[i] = ArmDevice.storage.joints_pos[i] + ArmDevice.storage.speeds[i] * dt
    return Joints(*new_joints)

def get_pos_field(resp):
    '''Gets the rotor position from a GetRotorPosition request'''
    try:
        position = resp['GetRotorPosition']['rotor_pos']
        return position
    except KeyError:
        return None

async def get_positions():
    ''' Returns an updated Joints object with the current arm positions'''
    new_joints = list(ArmDevice.storage.joints_pos)
    for i, device in enumerate(["armShoulder", "armElbow", "armWristPitch"]):
        resp = await ArmDevice.request('USBManager', device, {'GetRotorPosition':0})
        # log.debug(resp)
        if resp == 'no such endpoint':
            continue
        reading = get_pos_field(resp)
        if reading is not None:
            # if device == "d_armShoulder" and reading < 180:
            #     # The shoulder joint's magnet happens to be orientated
            #     # that the encoder flips from 360 to 0 partway through
            #     # the rotation.
            #     reading += 360
            reading += ArmDevice.storage.joint_offsets[device]
            new_joints[i+1] = round(math.radians(reading), 3) #Convert to radians
        else:
            log.warning("Could not read joint position {}".format(device))
    return Joints(*new_joints)

@ArmDevice.every(dt)
async def loop():
    """
    The main update loop. This reads positions from the encoders,
    and sends the current movement commands to the motors.
    """
    # ArmDevice.storage.joints_pos = await get_positions() # Use this if encoders are wired up.
    # ArmDevice.storage.joints_pos = simulate_positions() # Use this for testing without position feedback.
    log.debug("command: {}".format(ArmDevice.storage.command))
    ArmDevice.storage.controller.user_command(ArmDevice.storage.mode, *ArmDevice.storage.command)
    ArmDevice.storage.speeds = ArmDevice.storage.controller.update_duties(ArmDevice.storage.joints_pos)

    # publish speeds/duty cycles here
    log.debug("joints_pos: {}".format(ArmDevice.storage.joints_pos))
    log.debug("speeds: {}".format(ArmDevice.storage.speeds))
    await send_duties()

async def send_duties():
    ''' Tell each motor controller to turn on motors'''
    await ArmDevice.publish('armBase', {'SetRPM':int(base_erpm*ArmDevice.storage.speeds[0])})
    await ArmDevice.publish('armShoulder', {'SetRPM':int(shoulder_erpm*ArmDevice.storage.speeds[1])})
    if isinstance(ArmDevice.storage.mode, ManualControl):
        # let manual conrol go a bit faster.
        await ArmDevice.publish('armElbow', {'SetRPM':int(manual_elbow_erpm*ArmDevice.storage.speeds[2])})
    else:
        await ArmDevice.publish('armElbow', {'SetRPM':int(elbow_erpm*ArmDevice.storage.speeds[2])})
    await ArmDevice.publish('armForearmRot', {'SetRPM':int(3000*ArmDevice.storage.speeds[3])})
    await ArmDevice.publish('armWristPitch', {'SetRPM':int(wrist_pitch_erpm*ArmDevice.storage.speeds[4])})
    await ArmDevice.publish('armWristRot', {'SetDutyCycle':int(1e5*ArmDevice.storage.speeds[5])})
    await ArmDevice.publish('armGripperOpen', {'SetDutyCycle':int(1e5*ArmDevice.storage.speeds[6])})

@ArmDevice.on('*/controller{}/joystick1'.format(CONTROLLER_NUMBER))
async def on_joystick1(event, data):
    ''' Shoulder joint, and radius control.'''
    y_axis = data[1]
    if isinstance(ArmDevice.storage.mode, ManualControl):
        y_axis *= shoulder_max_speed
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

@ArmDevice.on('*/controller{}/joystick2'.format(CONTROLLER_NUMBER))
async def on_joystick2(event, data):
    ''' Elbow joints and z/height control'''
    y_axis = data[1]
    if isinstance(ArmDevice.storage.mode, ManualControl):
        y_axis *= -1*elbow_max_speed
        if y_axis > elbow_min_speed or y_axis < -elbow_min_speed:
            armY_ElbowSpeed = y_axis
        else:
            armY_ElbowSpeed = 0
        ArmDevice.storage.command[2] = armY_ElbowSpeed
    elif isinstance(ArmDevice.storage.mode, PlanarControl):
        y_axis = -1*(y_axis * height_max_speed)
        if y_axis > height_min_speed or y_axis < -height_min_speed:
            height_speed = y_axis
        else:
            height_speed = 0
        ArmDevice.storage.command[1] = height_speed

@ArmDevice.on('*/controller{}/dpad'.format(CONTROLLER_NUMBER))
async def on_dpad(event, data):
    """
    Wrist rotation, and wrist pitch.
    """
    x_axis = data[0]
    y_axis = data[1]
    ArmDevice.storage.command[4] = y_axis*wrist_pitch_speed
    ArmDevice.storage.command[5] = x_axis*gripper_rotation_speed

@ArmDevice.on('*/controller{}/trigger'.format(CONTROLLER_NUMBER))
async def on_trigger(event, data):
    """
    Base rotation.
    """
    armBaseSpeed = data*base_max_speed
    if -base_min_speed < armBaseSpeed < base_min_speed:
        armBaseSpeed = 0
    if isinstance(ArmDevice.storage.mode, ManualControl):
        ArmDevice.storage.command[0] = armBaseSpeed
    elif isinstance(ArmDevice.storage.mode, PlanarControl):
        ArmDevice.storage.command[2] = armBaseSpeed

@ArmDevice.on('*/controller{}/buttonB_down'.format(CONTROLLER_NUMBER))
async def on_buttonB_down(event, data):
    """
    Toggles between Inverse Kinematics and manual control modes.
    """
    if isinstance(ArmDevice.storage.mode, ManualControl):
        ArmDevice.storage.mode = PlanarControl()
        log.info("PlanarControl")
    else:
        ArmDevice.storage.mode = ManualControl()
        log.info("ManualControl")

# The mappings for button A and Y are broken into button down and up
# events because we're using two independant buttons to set the same
# variable. If we just watched the button state, the button that is
# released would keep setting the gripper speed at 0.
@ArmDevice.on('*/controller{}/buttonA_down'.format(CONTROLLER_NUMBER))
async def on_buttonA_down(event, data):
    """
    Open/close the end effector.
    """
    ArmDevice.storage.command[6] = gripper_open_speed

@ArmDevice.on('*/controller{}/buttonA_up'.format(CONTROLLER_NUMBER))
async def on_buttonA_up(event,data):
    ArmDevice.storage.command[6] = 0

@ArmDevice.on('*/controller{}/buttonY_up'.format(CONTROLLER_NUMBER))
async def on_buttonY_up(event,data):
    ArmDevice.storage.command[6] = 0

@ArmDevice.on('*/controller{}/buttonY_down'.format(CONTROLLER_NUMBER))
async def on_buttonY_down(event, data):
    ArmDevice.storage.command[6] = -gripper_open_speed

@ArmDevice.on('*/controller{}/bumperR_down'.format(CONTROLLER_NUMBER))
async def on_buttonA_down(event, data):
    """
    Rotate the forearm. Not really used at the moment.
    """
    ArmDevice.storage.command[3] = forearm_roll_speed

@ArmDevice.on('*/controller{}/bumperR_up'.format(CONTROLLER_NUMBER))
async def on_buttonA_up(event,data):
    ArmDevice.storage.command[3] = 0

@ArmDevice.on('*/controller{}/bumperL_up'.format(CONTROLLER_NUMBER))
async def on_buttonY_up(event,data):
    ArmDevice.storage.command[3] = 0

@ArmDevice.on('*/controller{}/bumperL_down'.format(CONTROLLER_NUMBER))
async def on_buttonY_down(event, data):
    ArmDevice.storage.command[3] = -forearm_roll_speed


setup(ArmDevice.storage)
ArmDevice.start()
ArmDevice.wait()
