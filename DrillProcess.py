from robocluster import Device
import config
log = config.getLogger()

#CONSTANTS
DRILL_RAISE_DUTY_CYCLE = 0.4*1e5    # at 12 volts
DRILL_LOWER_DUTY_CYCLE = 0.3*1e5    # at 12 volts
ROTATION_SPEED = 2e3                # at 12 volts
DEADZONE = 0.2 #joysticks arent perfect
TRIG_DEADZONE = 0.05
controller_num = config.science_controller

#INITIALIZTION
DrillDevice = Device('ScienceModule', 'rover', network=config.network)
DrillDevice.storage.top_motor_movement = 'stopped'
DrillDevice.storage.bottom_motor_movement = 'stopped'
DrillDevice.storage.rotation_direction = 'stopped'

@DrillDevice.on('*/controller{}/joystick1'.format(controller_num))
async def set_top_movement(joystick1, data):
    top_move = DrillDevice.storage.top_motor_movement
    axis = data[1]

    if axis is None:
        return
    else:
        duty_cycle = axis * DRILL_RAISE_DUTY_CYCLE

    if -DEADZONE < axis < DEADZONE:
        top_move = 'stopped'
        await DrillDevice.publish('DrillBottom', {'SetDutyCycle': int(0)})
    else:
        top_move = 'moving'
        await DrillDevice.publish('DrillTop', {'SetDutyCycle': int(duty_cycle)})

    log.debug('setting top movement to {}'.format(top_move))

@DrillDevice.on('*/controller{}/joystick2.'.format(controller_num))
async def set_bottom_movement(joystick2, data):
    bot_move = DrillDevice.storage.bottom_motor_movement
    axis = data[1]

    if axis is None:
        return
    else:
        duty_cycle = axis * DRILL_RAISE_DUTY_CYCLE

    if -DEADZONE < axis < DEADZONE:
        bot_move = 'stopped'
        await DrillDevice.publish('DrillBottom', {'SetDutyCycle': int(0)})
    else:
        bot_move = 'moving'
        await DrillDevice.publish('DrillBottom', {'SetDutyCycle': int(duty_cycle)})

    log.debug('setting bottom movement to {}'.format(bot_move))

@DrillDevice.on('*/controller{}/trigger.'.format(controller_num))
async def set_drill_rotation(trigger, data):
    rot_move = DrillDevice.storage.rotation_direction
    axis = data

    if axis is None:
        return
    else:
        duty_cycle = axis * ROTATION_SPEED

    if -TRIG_DEADZONE < axis < TRIG_DEADZONE:
        rot_move = 'stopped'
        await DrillDevice.publish('DrillSpin', {'SetRPM': int(0)})
    else:
        rot_move = 'moving'
        await DrillDevice.publish('DrillSpin', {'SetRPM': int(duty_cycle)})

    log.debug('Setting rotation movement to {}'.format(rot_move))

@DrillDevice.on('controller{}/{}_down'.format(controller_num, 'buttonB'))
async def hard_stop():

    top_move = DrillDevice.storage.top_motor_movement
    bot_move = DrillDevice.storage.bottom_motor_movement
    rot_move = DrillDevice.storage.rotation_direction

    await DrillDevice.publish('DrillTop', {'SetDutyCycle': int(0)})
    await DrillDevice.publish('DrillBottom', {'SetDutyCycle': int(0)})
    await DrillDevice.publish('DrillSpin', {'SetRPM': int(0)})

    DrillDevice.storage.top_motor_movement = 'stopped'
    DrillDevice.storage.bottom_motor_movement = 'stopped'
    DrillDevice.storage.rotation_direction = 'stopped'

    log.debug('setting top movement to {}'.format(top_move))
    log.debug('setting bottom movement to {}'.format(bot_move))
    log.debug('Setting rotation movement to {}'.format(rot_move))

DrillDevice.start()
DrillDevice.wait()








