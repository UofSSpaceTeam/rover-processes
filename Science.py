from robocluster import Device
import config
log = config.getLogger()

### CONSTANTS ###
DRILL_RAISE_DUTY_CYCLE = 0.2*1e5    # at 12 volts
DRILL_LOWER_DUTY_CYCLE = 0.1*1e5    # at 12 volts
ROTATION_SPEED = 2e3                # at 12 volts
SLEEP_DURATION = 0.1 #seconds

### INITIALIZTION ###
ScienceDevice = Device('ScienceModule', 'rover', network=config.network)
motor_movement_types = ['stopped', 'raising', 'lowering']
ScienceDevice.storage.top_motor_movement = 'stopped'
ScienceDevice.storage.bottom_motor_movement = 'stopped'
ScienceDevice.storage.rotation_direction = 'stopped'
ScienceDevice.storage.carousel_position = 'home'
ScienceDevice.storage.ready = 0
ScienceDevice.storage.top_raise_switch = 0
ScienceDevice.storage.top_lower_switch = 0
ScienceDevice.storage.bottom_raise_switch = 0
ScienceDevice.storage.bottom_lower_switch = 0
ScienceDevice.storage.sample_switch = 0
ScienceDevice.storage.empty_switch = 0

### FUNCTIONS ###
async def set_top_movement(mov):
    if mov == 'stopped':
        await ScienceDevice.publish('DrillTop', {'SetDutyCycle':int(0)})
    elif mov == 'raising':
        await ScienceDevice.publish('DrillTop', {'SetDutyCycle':int(DRILL_RAISE_DUTY_CYCLE)})
    elif mov == 'lowering':
        await ScienceDevice.publish('DrillTop', {'SetDutyCycle':int((-1)*DRILL_RAISE_DUTY_CYCLE)})
    else:
        log.error('motor movement invalid value: {}'.format(mov))
        return
    log.debug('Setting Top movement to {}'.format(mov))
    ScienceDevice.top_motor_movement = mov
    await ScienceDevice.sleep(SLEEP_DURATION)

async def set_bottom_movement(mov):
    if mov == 'stopped':
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle':int(0)})
    elif mov == 'raising':
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle':int(DRILL_RAISE_DUTY_CYCLE)})
    elif mov == 'lowering':
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle':int((-1)*DRILL_RAISE_DUTY_CYCLE)})
    else:
        log.error('motor movement invalid value: {}'.format(mov))
        return
    log.debug('Setting Bottom movement to {}'.format(mov))
    ScienceDevice.bottom_motor_movement = mov
    await ScienceDevice.sleep(SLEEP_DURATION)

async def set_drill_rotation(mov):
    if mov == 'stopped':
        await ScienceDevice.publish('DrillSpin', {'SetRPM':int(0)})
    elif mov == 'digging':
        await ScienceDevice.publish('DrillSpin', {'SetRPM':int(ROTATION_SPEED)})
    elif mov == 'retracting':
        await ScienceDevice.publish('DrillSpin', {'SetRPM':int((-1)*ROTATION_SPEED)})
    else:
        log.error('motor movement invalid value: {}'.format(mov))
        return
    log.debug('Setting Rotation movement to {}'.format(mov))
    ScienceDevice.rotation_direction = mov
    await ScienceDevice.sleep(SLEEP_DURATION)

async def hard_stop():
    await set_top_movement('stopped')
    await set_bottom_movement('stopped')
    await set_drill_rotation('stopped')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})

async def go_home():
    await hard_stop()
    while ScienceDevice.storage.top_raise_switch == False:
        await set_top_movement('raising')
    await set_top_movement('stopped')
    while ScienceDevice.storage.bottom_raise_switch == False:
        await set_bottom_movement('raising')
    await set_bottom_movement('stopped')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})

async def take_sample():
    log.debug(ScienceDevice.storage.carousel_position)
    while ScienceDevice.storage.carousel_position == 'home' and ScienceDevice.storage.top_lower_switch == 0:
        await set_top_movement('lowering')
    log.debug("Top lowered")
    await set_top_movement('stopped')
    await set_drill_rotation('digging')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
    while ScienceDevice.storage.carousel_position == 'home' and ScienceDevice.storage.bottom_lower_switch == 0:
        await set_bottom_movement('lowering')
    log.debug('Bottom lowered')
    await set_bottom_movement('stopped')
    await set_drill_rotation('retracting')
    while ScienceDevice.storage.carousel_position == 'home' and ScienceDevice.storage.bottom_raise_switch == 0:
        await set_bottom_movement('raising')
    log.debug('Bottom raised')
    await set_bottom_movement('stopped')
    await set_drill_rotation('stopped')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
    while ScienceDevice.storage.carousel_position == 'home' and ScienceDevice.storage.top_raise_switch == 0:
        await set_top_movement('raising')
    log.debug('Top raised')
    await set_top_movement('stopped')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})

async def deposit_sample():
    if ScienceDevice.storage.carousel_position == 'sample':
        await set_drill_rotation('stopped')
        for count in range (2):
            while ScienceDevice.storage.sample_switch == 0:
                await set_top_movement('lowering')
            await set_top_movement('stopped')
            await ScienceDevice.sleep(5)
            while ScienceDevice.storage.top_raise_switch == 0:
                await set_top_movement('raising')
            await set_top_movement('stopped')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})

async def empty_drill():
    if ScienceDevice.storage.carousel_position == 'empty':
        await set_drill_rotation('stopped')
        while ScienceDevice.storage.empty_switch == False:
            await set_top_movement('lowering')
        await set_top_movement('stopped')
        await ScienceDevice.sleep(10)
        while ScienceDevice.storage.top_raise_switch == False:
            await set_top_movement('raising')
        await set_top_movement('stopped')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})

# async def wait():
#     while ScienceDevice.storage.ready == 0:
#         await ScienceDevice.sleep(1)

@ScienceDevice.on('*/science_ready')
async def arduino_ready(event, data):
    ScienceDevice.storage.ready = data
    # if data == 0:
    #     ScienceDevice.task(wait)

@ScienceDevice.on('*/carousel_position')
async def carousel_position(event, data):
    log.debug('Carousel position {}'.format(data))
    ScienceDevice.storage.carousel_position = data

@ScienceDevice.on('*/science_limit_switches')
async def switches(event, data):
    log.debug('Updating limit switches {}'.format(data))
    ScienceDevice.storage.top_raise_switch = data[0]
    ScienceDevice.storage.top_lower_switch = data[1]
    ScienceDevice.storage.bottom_raise_switch = data[2]
    ScienceDevice.storage.bottom_lower_switch = data[3]
    ScienceDevice.storage.sample_switch = data[4]
    ScienceDevice.storage.empty_switch = data[5]

@ScienceDevice.on('*/StartScience')
async def start_sci(event, data):
    log.info('starting science')
    # await ScienceDevice.sleep(2)
    # await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
    log.info('Sent science to start')
    ScienceDevice.task(take_sample)

@ScienceDevice.on('*/take_sample')
async def sample(event, data):
    log.debug('got take sample')
    if data == 1:
        log.debug('Running take_sample function')
        ScienceDevice.task(take_sample)

@ScienceDevice.on('*/deposit_sample')
async def deposit(event, data):
    if data == 1:
        log.debug('Running deposit_sample')
        ScienceDevice.task(deposit_sample)

@ScienceDevice.on('*/empty_sample')
async def empty(event, data):
    if data == 1:
        log.debug('Running empty_drill')
        ScienceDevice.task(empty_drill)

ScienceDevice.start()
ScienceDevice.wait()

