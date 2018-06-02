#   Process to move drill in order to take soil sample, deposit sample, and empty the rest of the drill
#   Should not be able to operate unless carousel is in certain positions
#   There should be a check added to make sure the rover isn't moving, and to block rover movement while drill is extended
#   Constants still need to be set
#   top motor = motor controlling stage 1 of drill
#   bottom motor = motor controlling stage 2 of drill
#   should change motor naming scheme if time

from robocluster import Device
import time 

### CREATE DEVICE ###
drill = Device('Drill', 'rover')

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

### INITIALIZTION ###
drill.storage.top_motor_movement = 0
drill.storage.bottom_motor_movement = 0
drill.storage.rotating = False
drill.storage.top_distance = 0
drill.storage.bottom_distance = 0
drill.storage.carousel_position = 'EMPTY'

### FUNCTIONS ###
async def top_is_moving():
    if drill.storage.top_motor_movement == 1:
        return 'RAISING'
    elif drill.storage.top_motor_movement == 2:
        return 'LOWERING'
    else:
        return 'STOPPED'

async def bottom_is_moving():
    if drill.storage.bottom_motor_movement == 1:
        return 'RAISING'
    elif drill.storage.bottom_motor_movement == 2:
        return 'LOWERING'
    else:
        return 'STOPPED'

async def is_spinning():
    if drill.storage.rotating == True:
        return 'MOVING'
    else:
        return 'STOPPED'

async def hard_stop():
    stop_rotation()
    stop_bottom()
    stop_top()

async def raise_top():    
    drill.storage.top_motor_movement = 1
    await drill.publish('DrillTop', {'SetDutyCycle':int((-1)*DRILL_RAISE_DUTY_CYCLE)})
    await drill.sleep(0.01)

async def raise_bottom():
    drill.storage.bottom_motor_movement = 1
    await drill.publish('DrillBottom', {'SetDutyCycle':int((-1)*DRILL_RAISE_DUTY_CYCLE)})
    await drill.sleep(0.01)

async def lower_top():
    drill.storage.top_motor_movement = 2
    await drill.publish('DrillTop', {'SetDutyCycle':int(DRILL_LOWER_DUTY_CYCLE)})
    await drill.sleep(0.01)

async def lower_bottom():
    drill.storage.bottom_motor_movement = 2
    await drill.publish('DrillBottom', {'SetDutyCycle':int(DRILL_LOWER_DUTY_CYCLE)})
    await drill.sleep(0.01)

async def stop_top():
    await drill.publish('DrillTop', {'SetDutyCycle':0})
    drill.storage.top_motor_movement = 0
    await drill.sleep(0.01)

async def stop_bottom():
    await drill.publish('DrillBottom', {'SetDutyCycle':0})
    drill.storage.bottom_motor_movement = 0
    await drill.sleep(0.01)

async def start_rotation():
    drill.storage.rotating = True
    await drill.publish('DrillSpin', {'SetRPM':int(ROTATION_SPEED)})
    await drill.sleep(0.01)

async def stop_rotation():
    await drill.publish('DrillSpin', {'SetRPM':0})
    await drill.sleep(0.01)
    drill.storage.rotating = False

async def go_home():
    hard_stop()
    if drill.storage.home_switch == True:
        pass
    while drill.storage.home_switch == False:
        raise_bottom()
        raise_top()
    drill.storage.top_distance = 0
    drill.storage.bottom_distance = 0
    hard_stop()

async def take_sample():
    top_lower_start = time.time()
    while drill.storage.carousel_position == 'HOME' and drill.storage.top_distance <= TOP_VERT_DISTANCE:
        lower_top()
        #drill.storage.top_distance = TOP_ONE_ROT*(DRILL_SPEED/60)*(time.time() - top_lower_start)
        if drill.storage.distance_above_ground <= STOP_ABOVE_GROUND:
            stop_top()
            break

    bottom_lower_start = time.time()
    while drill.storage.carousel_position == 'HOME' and drill.storage.bottom_distance <= BOTTOM_VERT_DISTANCE:
        lower_bottom()
        #drill.storage.bottom_distance = BOTTOM_ONE_ROT*(DRILL_SPEED/60)*(time.time() - bottom_lower_start)
        if drill.storage.bottom_distance > START_ROT_DISTANCE:
            start_rotation()

    bottom_raise_start = time.time()
    while drill.storage.carousel_position == 'HOME' and drill.storage.bottom_distance >= 0:
        raise_bottom()
        #drill.storage.bottom_distance -= BOTTOM_ONE_ROT*(DRILL_SPEED/60)*(time.time() - bottom_raise_start)
        if drill.storage.home_switch == True:
            break
        if drill.storage.bottom_distance < START_ROT_DISTANCE:
            stop_rotation()
 
    top_raise_start = time.time()
    while drill.storage.carousel_position == 'HOME' and drill.storage.top_distance >= 0:
        raise_top()
        #drill.storage.top_distance -= TOP_ONE_ROT*(DRILL_SPEED/60)*(time.time() - top_raise_start)
        if drill.storage.home_switch == True:
            break

async def deposit_sample():
    if drill.storage.carousel_position == 'SAMPLE':
        await stop_rotation()
        for count in range (2):
            while drill.storage.top_distance <= SAMPLE_HOLDER_HEIGHT:
                await lower_top()
            time.sleep(5)
            while drill.storage.top_distance >= 0:
                raise_top()
                if drill.storage.home_switch == True:
                    break

async def empty_drill():
    if drill.storage.carousel_position == 'EMPTY':
        await stop_rotation()
        while drill.storage.top_distance <= SAMPLE_HOLDER_HEIGHT:
            await lower_top()
        await drill.sleep(5)
        while drill.storage.top_distance >= 0:
            await raise_top()
            if drill.storage.home_switch == True:
                break

async def wait():
    while drill.storage.carousel_position == 'AWAY':
        await drill.sleep(0.5)

### POLLS CAROUSEL POSITION ###
#@drill.on('*/carousel_home')
async def update_carousel_position(event, data):
    drill.storage.carousel_position = data

### BROADCASTS CURRENT DRILL STATE ###
@drill.on
async def broadcast_drill_state():
    await drill.publish('DrillSpinState', is_spinning())
    await drill.publish('DrillTopState', top_is_moving())
    await drill.publish('DrillBottomState', bottom_is_moving())
    await drill.publish('DrillTopPosition', drill.storage.top_distance)
    await drill.publish('DrillBottomPosition', drill.storage.bottom_distance)

### MAIN TASK ###
#@drill.task
async def main_task():
    await drill.sleep(2)
    await wait() #will receive go signal from science module
    await go_home()
    await take_sample()
    await go_home()
    await wait() # waits for carousel to be in sample postion and send go signal
    await deposit_sample()
    await go_home()
    await wait() # waits for carousel to be in empty position and send go signal
    await empty_drill()
    await go_home()

### TEST TASK ###
@drill.task
async def test_task():
    await drill.sleep(2)
    drill.storage.carousel_position = 'EMPTY'
    start = time.time()
    x = 0
    while (x - start) <= 4:
        await lower_top()
        x = time.time()
    await stop_top()
    await drill.sleep(3)
    while (x - start) <= 8:
        await raise_top()
        x = time.time()
        await drill.sleep(3)
    while (x - start) <= 15:
        await start_rotation()
        x = time.time()
    await stop_rotation()

drill.start()
drill.wait()







    

    





