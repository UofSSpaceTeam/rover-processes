from robocluster import Device

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
SLEEP_DURATION = 0.01 #seconds

ScienceDevice = Device('ScieneModue', 'rover')

### INITIALIZTION ###
motor_movement_types = ['stopped', 'raising', 'lowering']
ScienceDevice.storage.top_motor_movement = 'stopped'
ScienceDevice.storage.bottom_motor_movement = 'stopped'
ScienceDevice.storage.rotation_direction = 'stopped'
ScienceDevice.storage.top_distance = 0
ScienceDevice.storage.bottom_distance = 0
ScienceDevice.storage.carousel_position = 'empty'

### FUNCTIONS ###
async def set_top_movement(mov):
    if mov == 'stopped':
        await ScienceDevice.publish('DrillTop', {'SetDutyCycle'}:int(0))
    elif mov == 'raising':
        await ScienceDevice.publish('DrillTop', {'SetDutyCycle':int((-1)*DRILL_RAISE_DUTY_CYCLE)})
    elif mov == 'lowering':
        await ScienceDevice.publish('DrillTop', {'SetDutyCycle':int(DRILL_RAISE_DUTY_CYCLE)})
    else:
        log.error('motor movement invalid value: {}'.format(mov))
        return
    log.debug('Setting Top movement to {}'.format(mov))
    ScienceDevice.top_motor_movement = mov
    await ScienceDevice.sleep(SLEEP_DURATION)

async def set_bottom_movement(mov):
    if mov == 'stopped':
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle'}:int(0))
    elif mov == 'raising':
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle':int((-1)*DRILL_RAISE_DUTY_CYCLE)})
    elif mov == 'lowering':
        await ScienceDevice.publish('DrillBottom', {'SetDutyCycle':int(DRILL_RAISE_DUTY_CYCLE)})
    else:
        log.error('motor movement invalid value: {}'.format(mov))
        return
    log.debug('Setting Bottom movement to {}'.format(mov))
    ScienceDevice.bottom_motor_movement = mov
    await ScienceDevice.sleep(SLEEP_DURATION)

async def set_drill_rotation(mov):
    if mov == 'stopped':
        await ScienceDevice.publish('DrillSpin', {'SetRPM'}:int(0))
    elif mov == 'digging':
        await ScienceDevice.publish('DrillSpin', {'SetRPM':int(ROTATION_SPEED)})
    elif mov == 'retracting':
        await ScienceDevice.publish('DrillSpin', {'SetRPM':int(-1*ROTATION_SPEED)})
    else:
        log.error('motor movement invalid value: {}'.format(mov))
        return
    log.debug('Setting Rotation movement to {}'.format(mov))
    ScienceDevice.rotation_direction = mov
    await ScienceDevice.sleep(SLEEP_DURATION)

async hard_stop():
    await set_top_movement('stopped')
    await set_bottom_movement('stopped')
    await set_drill_rotation('stopped')

async def go_home():
    await hard_stop()
    while ScienceDevice.storage.home_switch == False:
        await set_top_movement('raising')
        await set_bottom_movement('raising')
    await hard_stop()
    ScienceDevice.storage.top_distance = 0
    ScienceDevice.storage.bottom_distance = 0

async def take_sample():
    top_lower_start = time.time()
    while ScienceDevice.storage.carousel_position == 'HOME' and ScienceDevice.storage.top_distance <= TOP_VERT_DISTANCE:
        await set_top_movement('lowering')
        #ScienceDevice.storage.top_distance = TOP_ONE_ROT*(DRILL_SPEED/60)*(time.time() - top_lower_start)
        if ScienceDevice.storage.distance_above_ground <= STOP_ABOVE_GROUND:
            await set_top_movement('stopped')
            break
    await set_top_movement('stopped')

    bottom_lower_start = time.time()
    while ScienceDevice.storage.carousel_position == 'HOME' and ScienceDevice.storage.bottom_distance <= BOTTOM_VERT_DISTANCE:
        await set_bottom_movement('lowering')
        #ScienceDevice.storage.bottom_distance = BOTTOM_ONE_ROT*(DRILL_SPEED/60)*(time.time() - bottom_lower_start)
        if ScienceDevice.storage.bottom_distance > START_ROT_DISTANCE:
            await set_drill_rotation('drilling')
    await set_bottom_movement('stopped')

    bottom_raise_start = time.time()
    while ScienceDevice.storage.carousel_position == 'HOME' and ScienceDevice.storage.bottom_distance >= 0:
        await set_bottom_movement('raising')
        #ScienceDevice.storage.bottom_distance -= BOTTOM_ONE_ROT*(DRILL_SPEED/60)*(time.time() - bottom_raise_start)
        if ScienceDevice.storage.home_switch == True:
            break
        if ScienceDevice.storage.bottom_distance < START_ROT_DISTANCE:
            await set_drill_rotation('stopped')
    await set_bottom_movement('stopped')

    top_raise_start = time.time()
    while ScienceDevice.storage.carousel_position == 'HOME' and ScienceDevice.storage.top_distance >= 0:
        raise_top()
        await set_top_movement('raising')
        #ScienceDevice.storage.top_distance -= TOP_ONE_ROT*(DRILL_SPEED/60)*(time.time() - top_raise_start)
        if ScienceDevice.storage.home_switch == True:
            break
    await set_top_movement('stopped')

async def deposit_sample():
    if ScienceDevice.storage.carousel_position == 'SAMPLE':
        await set_drill_rotation('stopped')
        for count in range (2):
            while ScienceDevice.storage.top_distance <= SAMPLE_HOLDER_HEIGHT:
                await set_top_movement('lowering')
            await ScienceDevice.sleep(5)
            while ScienceDevice.storage.top_distance >= 0:
                await set_top_movement('raising')
                if ScienceDevice.storage.home_switch == True:
                    break

async def empty_drill():
    if ScienceDevice.storage.carousel_position == 'EMPTY':
        await set_drill_rotation('stopped')
        while ScienceDevice.storage.top_distance <= SAMPLE_HOLDER_HEIGHT:
            await set_top_movement('lowering')
        await ScienceDevice.sleep(5)
        while ScienceDevice.storage.top_distance >= 0:
            await set_top_movement('raising')
            if ScienceDevice.storage.home_switch == True:
                break

async def wait():
    while ScienceDevice.storage.carousel_position == 'AWAY':
        await ScienceDevice.sleep(0.5)

@ScienceDevice.task
async def start_sci():
    await ScienceDevice.sleep(2)
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
    print('Sent science to start')


ScienceDevice.start()
ScienceDevice.wait()
