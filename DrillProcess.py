rom robocluster import Device
import time
import pygame
import config
log = config.getLogger()

### CREATE DEVICE ###
drill = Device('Drill', 'rover')
JoystickDevice = Device('JoystickDevice', 'rover', network=config.network)

### CONSTANTS - NOT ALL SET AT THE MOMENT ###
TOP_VERT_DISTANCE = 150 #350 # mm
TOP_ONE_ROT = 65.7 # mm
BOTTOM_ONE_ROT = 0
BOTTOM_VERT_DISTANCE = 0
START_ROT_DISTANCE = 0
DRILL_RAISE_DUTY_CYCLE = 0.4*1e5    # at 12 volts
DRILL_LOWER_DUTY_CYCLE = 0.3*1e5    # at 12 volts
ROTATION_SPEED = 2e3                # at 12 volts
SAMPLE_HOLDER_HEIGHT = 100
STOP_ABOVE_GROUND = 1 # mm
DEADZONE = 0.2 #analog joysticks arent perfect
TRIG_DEADZONE = 0.05

done = False


'''### INITIALIZTION ###
drill.storage.top_motor_movement = 0
drill.storage.bottom_motor_movement = 0
drill.storage.rotating = False
drill.storage.top_distance = 0
drill.storage.bottom_distance = 0
drill.storage.carousel_position = 'EMPTY'''

### CONSTANTS ###
DRILL_RAISE_DUTY_CYCLE = 0.2*1e5    # at 12 volts
DRILL_LOWER_DUTY_CYCLE = 0.1*1e5    # at 12 volts
ROTATION_SPEED = 2e3                # at 12 volts
SLEEP_DURATION = 0.1 #seconds
controller_num = config.science_controller

### INITIALIZTION ###
ScienceDevice = Device('ScienceModule', 'rover', network=config.network)
motor_movement_types = ['stopped', 'raising', 'lowering']

ScienceDevice.storage.top_motor_movement = top_move
ScienceDevice.storage.bottom_motor_movement = bot_move
ScienceDevice.storage.rotation_direction = rot_move
ScienceDevice.storage.carousel_position = 'home'
ScienceDevice.storage.ready = 0
ScienceDevice.storage.top_raise_switch = 0
ScienceDevice.storage.top_lower_switch = 0
ScienceDevice.storage.bottom_raise_switch = 0
ScienceDevice.storage.bottom_lower_switch = 0
ScienceDevice.storage.sample_switch = 0
ScienceDevice.storage.empty_switch = 0

top_move = 'stopped'
bot_move = 'stopped'
rot_move = 'stopped'


@ScienceDevice.on('*/controller{}/joystick1'.format(controller_num))
async def top_motor_movement(joystick1, data):
    global top_move
    axis = data[1]

    if axis is None:
        return
    else:
        duty_cycle = axis*DRILL_RAISE_DUTY_CYCLE

    if -DEADZONE < axis < DEADZONE:
        top_move = 'stopped'
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle': int(0)})
    elif axis > DEADZONE:
        top_move = 'raising'
        await ScienceDevice.publish('DrillTop', {'SetDutyCycle': int(duty_cycle)})
    elif axis < -DEADZONE:
        top_move = 'lowering'
        await ScienceDevice.publish('DrillTop', {'SetDutyCycle': int(duty_cycle)})
    else:
        await ScienceDevice.publish('DrillTop', {'SetDutyCycle': int(0)})

    #ScienceDevice.top_motor_movement = top_move
    log.debug('setting top movement to {}'.format(top_move))
    await ScienceDevice.sleep(SLEEP_DURATION)

@ScienceDevice.on('*/controller{}/joystick2.'.format(controller_num))
async def top_motor_movement(joystick2, data):
    global bot_move
    axis = data[1]

    if axis is None:
            return
    else:
        duty_cycle = axis * DRILL_RAISE_DUTY_CYCLE

    if -DEADZONE < axis < DEADZONE:
        bot_move = 'stopped'
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle': int(0)})
    elif axis > DEADZONE:
        bot_move = 'raising'
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle': int(duty_cycle)})
    elif axis < -DEADZONE:
        bot_move = 'lowering'
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle': int(duty_cycle)})
    else:
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle': int(0)})

    #ScienceDevice.bottom_motor_movement = bot_move
    log.debug('setting bottom movement to {}'.format(bot_move))
    await ScienceDevice.sleep(SLEEP_DURATION)

@ScienceDevice.on('*/controller{}/trigger.'.format(controller_num))
async def set_drill_rotation(trigger, data):
    global rot_move
    axis = data

    if axis is None:
        return
    else:
        duty_cycle = axis * ROTATION_SPEED

    if -TRIG_DEADZONE < axis < TRIG_DEADZONE:
        rot_move = 'stopped'
        await ScienceDevice.publish('DrillSpin', {'SetRPM': int(duty_cycle)})
    elif axis > TRIG_DEADZONE:
        rot_move = 'digging'
        await ScienceDevice.publish('DrillSpin', {'SetRPM': int(0)})
    elif axis < -TRIG_DEADZONE:
        rot_move = 'retracting'
        await ScienceDevice.publish('DrillSpin', {'SetRPM': int(duty_cycle)})
    else:
        log.error("motor rotation invalid value {}".format(rot_move))
        return

    log.debug('Setting rotation movement to {}'.format(rot_move))
    #ScienceDevice,rotation_direction = rot_move
    await ScienceDevice.sleep(SLEEP_DURATION)

@ScienceDevice.on('controller{}/{}_down'.format(controller_num, 'buttonB'))
async def hard_stop():

    global top_move, bot_move, rot_move

        top_move = 'stopped'
        bot_move = 'stopped'
        rot_move = 'stopped'
        await ScienceDevice.publish('ScienceArduino', {'enable_science': 1})


ScienceDevice.start()
ScienceDevice.wait()








