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
DrillDevice = Device('DrillDevice', 'rover', network=config.network)
DrillDevice.storage.top_motor_movement = 'stopped'
DrillDevice.storage.bottom_motor_movement = 'stopped'
DrillDevice.storage.rotation_direction = 'stopped'

@DrillDevice.on('*/controller{}/joystick1'.format(controller_num))
async def set_top_movement(joystick1, data):
    axis = data[1]

    if axis is None:
        return
    else:
        duty_cycle = axis * DRILL_RAISE_DUTY_CYCLE

    if -DEADZONE < axis < DEADZONE:
        DrillDevice.storage.top_motor_movement = 'stopped'
        await DrillDevice.publish('DrillBottom', {'SetDutyCycle': int(0)})
    else:
        DrillDevice.storage.top_motor_movement = 'moving'
        await DrillDevice.publish('DrillTop', {'SetDutyCycle': int(duty_cycle)})

    log.debug('setting top movement to {}'.format(DrillDevice.storage.top_motor_movement))

@DrillDevice.on('*/controller{}/joystick2.'.format(controller_num))
async def set_bottom_movement(joystick2, data):
    axis = data[1]

    if axis is None:
        return
    else:
        duty_cycle = axis * DRILL_RAISE_DUTY_CYCLE

    if -DEADZONE < axis < DEADZONE:
        DrillDevice.storage.bottom_motor_movement = 'stopped'
        await DrillDevice.publish('DrillBottom', {'SetDutyCycle': int(0)})
    else:
        DrillDevice.storage.bottom_motor_movement = 'moving'
        await DrillDevice.publish('DrillBottom', {'SetDutyCycle': int(duty_cycle)})

    log.debug('setting bottom movement to {}'.format(DrillDevice.storage.bottom_motor_movement))

@DrillDevice.on('*/controller{}/trigger.'.format(controller_num))
async def set_drill_rotation(trigger, data):
    axis = data

    if axis is None:
        return
    else:
        duty_cycle = axis * ROTATION_SPEED

    if -TRIG_DEADZONE < axis < TRIG_DEADZONE:
        DrillDevice.storage.rotation_direction = 'stopped'
        await DrillDevice.publish('DrillSpin', {'SetRPM': int(0)})
    else:
        DrillDevice.storage.rotation_direction = 'moving'
        await DrillDevice.publish('DrillSpin', {'SetRPM': int(duty_cycle)})

    log.debug('Setting rotation movement to {}'.format(DrillDevice.storage.rotation_direction))

@DrillDevice.on('controller{}/{}_down'.format(controller_num, 'buttonB'))
async def hard_stop():

    await DrillDevice.publish('DrillTop', {'SetDutyCycle': int(0)})
    await DrillDevice.publish('DrillBottom', {'SetDutyCycle': int(0)})
    await DrillDevice.publish('DrillSpin', {'SetRPM': int(0)})

    DrillDevice.storage.top_motor_movement = 'stopped'
    DrillDevice.storage.bottom_motor_movement = 'stopped'
    DrillDevice.storage.rotation_direction = 'stopped'

    log.debug('setting top movement to {}'.format(DrillDevice.storage.top_motor_movement))
    log.debug('setting bottom movement to {}'.format(DrillDevice.storage.bottom_motor_movement))
    log.debug('Setting rotation movement to {}'.format(DrillDevice.storage.rotation_direction))

@ScienceDevice.on('*/science_limit_switches')
async def switches(event, data):
    log.debug('Updating limit switches {}'.format(data))
    ScienceDevice.storage.top_raise_switch = data[0]
    ScienceDevice.storage.top_lower_switch = data[1]
    ScienceDevice.storage.bottom_raise_switch = data[2]
    ScienceDevice.storage.bottom_lower_switch = data[3]
    ScienceDevice.storage.sample_switch = data[4]
    ScienceDevice.storage.empty_switch = data[5]

DrillDevice.start()
DrillDevice.wait()








