from Robocluster import Device

### CONSTANTS ###
DRILL_RAISE_DUTY_CYCLE = 0.4*1e5    # at 12 volts
DRILL_LOWER_DUTY_CYCLE = 0.3*1e5    # at 12 volts
ROTATION_SPEED = 2e3                # at 12 volts
SLEEP_DURATION = 0.01 #seconds

### INITIALIZTION ###
ScienceDevice = Device('ScienceModule', 'rover')
motor_movement_types = ['stopped', 'raising', 'lowering']
ScienceDevice.storage.top_motor_movement = 'stopped'
ScienceDevice.storage.bottom_motor_movement = 'stopped'
ScienceDevice.storage.rotation_direction = 'stopped'
ScienceDevice.storage.carousel_position = 'home'
ScienceDevice.storage.ready = 0

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
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
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
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
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
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
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
    while ScienceDevice.storage.carousel_position == 'home' and ScienceDevice.storage.top_lower_switch == False:
        await set_top_movement('lowering')
    await set_top_movement('stopped')
    await set_drill_rotation('digging')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
    while ScienceDevice.storage.carousel_position == 'home' and ScienceDevice.storage.bottom_lower_switch == False:
        await set_bottom_movement('lowering')
    await set_bottom_movement('stopped')
    await set_drill_rotation('retracting')
    while ScienceDevice.storage.carousel_position == 'home' and ScienceDevice.storage.bottom_raise_switch == False:
        await set_bottom_movement('raising')
    await set_bottom_movement('stopped')
    await set_drill_rotation('stopped')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
    while ScienceDevice.storage.carousel_position == 'home' and ScienceDevice.storage.top_raise_switch == False:
        await set_top_movement('raising')
    await set_top_movement('stopped')
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})

async def deposit_sample():
    if ScienceDevice.storage.carousel_position == 'sample':
        await set_drill_rotation('stopped')
        for count in range (2):
            while ScienceDevice.storage.sample_switch == False:
                await set_top_movement('lowering')
            await set_top_movement('stopped')
            await ScienceDevice.sleep(5)
            while ScienceDevice.storage.top_raise_switch == False:
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

async def wait():
    while ScienceDevice.storage.ready == 0:
        ScienceDevice.sleep(1)

@ScienceDevice.every('*/science_ready')
async def arduino_ready(event, data):
    ScienceDevice.storage.ready = data
    if data == 0:
        await wait()

@ScienceDevice.every('*/carousel_position')
async def carousel_position(event, data):
    ScienceDevice.storage.carousel_position = data

@ScienceDevice.every('*/science_limit_switches')
async def switches(event, data):
    ScienceDevice.storage.top_raise_switch = data[1]
    ScienceDevice.storage.top_lower_switch = data[2]
    ScienceDevice.storage.bottom_raise_switch = data[3]
    ScienceDevice.storage.bottom_lower_switch = data[4]
    ScienceDevice.storage.sample_switch = data[5]
    ScienceDevice.storage.empty_switch = data[6]

@ScienceDevice.task
async def start_sci():
    await ScienceDevice.sleep(2)
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
    print('Sent science to start')
    await wait()

@ScienceDevice.on('*/take_sample')
async def sample(event, data):
    if data == 1:
        await take_sample()

@ScienceDevice.on('*/deposit_sample')
async def deposit(event, data):
    if data == 1:
        await deposit_sample()

@ScienceDevice.on('*/empty_sample')
async def empty(event, data):
    if data == 1:
        empty_drill()

ScienceDevice.start()
ScienceDevice.wait()

