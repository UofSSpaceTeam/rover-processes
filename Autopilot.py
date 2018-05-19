import math

from robocluster import Device

import config
log = config.getLogger()

Autopilot = Device('Autopilot', 'rover', network=config.network)

LOOP_PERIOD = 0.1
MIN_WHEEL_RPM = 1000
MAX_SPEED = 2 # m/s
BEARING_THRESH = 10 # degrees
GPS_DISTANCE_THRESH = 5 # meters
BALL_HORIZONTAL_THRESH = 20 # pixels?
BALL_DISTANCE_THRESH = 1

###### Initialization ########

Autopilot.storage.state = None
Autopilot.storage.enabled = False
Autopilot.storage.waypoints = []


######## State Machine ######
async def waiting():
    if Autopilot.storage.enabled:
        Autopilot.storage.waypoints = await Autopilot.request( 'Navigation', 'waypoints')
        log.debug("Waypoints: {}".format(Autopilot.storage.waypoints))
        if len(Autopilot.storage.waypoints) > 0:
            Autopilot.storage.state = drive_to_target
    else:
        await Autopilot.send('DriveSystem', 'Stop', 0)
        await Autopilot.send('DriveSystem', 'Stop', 0)

async def drive_to_target():
    if Autopilot.storage.enabled:
        position = await Autopilot.request('Navigation', 'RoverPosition')
        if position == [0,0]:
            return # initialization state
        waypoints = Autopilot.storage.waypoints
        if len(Autopilot.storage.waypoints) > 0:
            # Make sure we're pointed in the right direction
            bearing = await Autopilot.request('Navigation', 'bearing', position, waypoints[0])
            bearing = bearing%360
            heading = await Autopilot.request('Navigation', 'RoverHeading')
            distance = await Autopilot.request('Navigation', 'distance', position, waypoints[0])
            a = bearing - heading
            a = (a+180)%360 - 180 # find smallest angle difference
            if abs(a) > BEARING_THRESH + distance/10:
                if a >= 0: # Turn right
                    log.info('Turn right')
                    await Autopilot.send('DriveSystem', 'RotateRight', MAX_SPEED/40)
                else: # Turn left
                    log.info('Turn left')
                    await Autopilot.send('DriveSystem', 'RotateLeft', MAX_SPEED/40)
                return
            log.info("distance from {} to {} = {}".format(position, waypoints[0], distance))
            if distance > GPS_DISTANCE_THRESH:
                await Autopilot.send('DriveSystem', 'DriveForward', MAX_SPEED)
            else:
                log.info('!!!!! Searching for ball !!!!!!!')
                Autopilot.storage.state = search_for_ball
                # Autopilot.storage.enabled = False
                # await Autopilot.send('DriveSystem', 'Stop', 0)
                # await Autopilot.publish("Autopilot", False)  # update WebUI
                # await Autopilot.publish("TargetReached", True)
                # Autopilot.storage.state = waiting
    else:
        Autopilot.storage.state = waiting


async def search_for_ball():
    if Autopilot.storage.enabled:
        ball_coords = await Autopilot.request('Navigation', 'BallPosition')
        if ball_coords is not None:
            Autopilot.storage.state = drive_to_ball
        else:
            await Autopilot.send('DriveSystem', 'RotateRight', MAX_SPEED/40)
    else:
        Autopilot.storage.state = waiting

async def drive_to_ball():
    if Autopilot.storage.enabled:
        ball_coords = await Autopilot.request('Navigation', 'BallPosition')
        if ball_coords is not None:
            if ball_coords['x'] > BALL_HORIZONTAL_THRESH:
                log.info('Turn right')
                await Autopilot.send('DriveSystem', 'RotateRight', MAX_SPEED/40)
            elif ball_coords['x'] < -BALL_HORIZONTAL_THRESH:
                log.info('Turn left')
                await Autopilot.send('DriveSystem', 'RotateLeft', MAX_SPEED/40)
            else:
                if ball_coords['distance'] >= BALL_DISTANCE_THRESH:
                    log.info('Driving to the ball: {}', ball_coords['distance'])
                    await Autopilot.send('DriveSystem', 'DriveForward', MAX_SPEED)
                else:
                    log.info('!!!!!! Got to the ball !!!!!!!!')
                    Autopilot.storage.enabled = False
                    await Autopilot.send('DriveSystem', 'Stop', 0)
                    await Autopilot.publish("Autopilot", False)  # update WebUI
                    await Autopilot.publish("TargetReached", True)
                    Autopilot.storage.state = waiting
        else:
            Autopilot.storage.state = search_for_ball
    else:
        Autopilot.storage.state = waiting


Autopilot.storage.state = waiting
@Autopilot.every(LOOP_PERIOD)
async def state_machine():
    log.debug(Autopilot.storage.state.__name__)
    await Autopilot.storage.state()


######## Callbacks ###########

@Autopilot.on('*/Autopilot')
def enable_autopilot(event, data):
    log.info('Setting Autopilot to {}'.format(data))
    Autopilot.storage.enabled = data

Autopilot.start()
Autopilot.wait()
