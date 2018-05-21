#   Process to move drill in order to take soil sample, deposit sample, and empty the rest of the drill
#   Should not be able to operate unless carousel is in certain positions
#   There should be a check added to make sure the rover isn't moving, and to block rover movement while drill is extended
#   Constants still need to be set

from robocluster import Device
import time 

### CREATE DEVICE ###
drill = Device('Drill', 'rover')

### CONSTANTS ###
TOP_VERT_DISTANCE = 350 # mm
TOP_ONE_ROT = 65.7 # mm
BOTTOM_VERT_DISTANCE = 0
START_ROT_DISTANCE = 0
DRILL_SPEED = 0
ROTATION_SPEED = 0
SAMPLE_HOLDER_HEIGHT = 0
STOP_ABOVE_GROUND = 1 # mm

### INITIALIZTION ###
drill.storage.top_motor_movement = 0
drill.storage.bottom_motor_movement = 0
drill.storage.rotating = False
drill.storage.top_distance = 0
drill.storage.bottom_distance = 0

### METHODS, SORT OF ###
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
    await drill.publish('DrillTop', {'SetRPM':(-1)*DRILL_SPEED})

async def raise_bottom():
    drill.storage.bottom_motor_movement = 1
    await drill.publish('DrillBottom', {'SetRPM':(-1)*DRILL_SPEED})

async def lower_top():
        drill.storage.top_motor_movement = 2
        await drill.publish('DrillTop', {'SetRPM':DRILL_SPEED})

async def lower_bottom():
    drill.storage.bottom_motor_movement = 2
    await drill.publish('DrillBottom', {'SetRPM':DRILL_SPEED})

async def stop_top():
    await drill.publish('DrillTop', {'SetRPM':0})
    drill.storage.top_motor_movement = 0

async def stop_bottom():
    await drill.publish('DrillBottom', {'SetRPM':0})
    drill.storage.bottom_motor_movement = 0

async def start_rotation():
    drill.storage.rotating = True
    await drill.publish('DrillSpin', {'SetRPM':ROTATION_SPEED})

async def stop_rotation():
    await drill.publish('DrillSpin', {'SetRPM':0})
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
    start = time.time()
    while drill.storage.carousel_position == 'HOME' and drill.storage.top_distance <= TOP_VERT_DISTANCE:
        lower_top()
        drill.storage.top_distance = TOP_ONE_ROT*(DRILL_SPEED/60)*(start - time.time())
        # check with Carl if this will work
        if drill.storage.distance_above_ground <= STOP_ABOVE_GROUND:
            stop_top()
            break

    while drill.storage.carousel_position == 'HOME' and drill.storage.bottom_distance <= BOTTOM_VERT_DISTANCE:
        lower_bottom()
        if drill.storage.bottom_distance > START_ROT_DISTANCE:
            start_rotation()

    while drill.storage.carousel_position == 'HOME' and drill.storage.bottom_distance >= 0:
        raise_bottom()
        if drill.storage.home_switch == True:
            break
        if drill.storage.bottom_distance < START_ROT_DISTANCE:
            stop_rotation()
 
    while drill.storage.carousel_position == 'HOME' and drill.storage.top_distance >= 0:
        raise_top()
        if drill.storage.home_switch == True:
            break

async def deposit_sample():
    if drill.storage.carousel_position == 'SAMPLE':
        stop_rotation()
        for count in range (2):
            while drill.storage.top_distance <= SAMPLE_HOLDER_HEIGHT:
                lower_top()
            time.sleep(5)
            while drill.storage.top_distance >= 0:
                raise_top()
                if drill.storage.home_switch == True:
                    break

async def empty_drill():
    if drill.storage.carousel_position == 'EMPTY':
        stop_rotation()
        while drill.storage.top_distance <= SAMPLE_HOLDER_HEIGHT:
            lower_top()
        time.sleep(5)
        while drill.storage.top_distance >= 0:
            raise_top()
            if drill.storage.home_switch == True:
                break

async def wait():
    while drill.storage.carousel_position == 'AWAY':
        time.sleep(0.5)

### POLLS CAROUSEL POSITION ###
@drill.on('*/carousel_home')
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

### MAIN LOOP, PERFORMS TASK ###
@drill.task
async def main_loop():
    wait()   
    go_home()
    take_sample()
    go_home()
    wait()
    deposit_sample()
    go_home()
    wait()
    empty_drill()
    go_home()

     






    

    





