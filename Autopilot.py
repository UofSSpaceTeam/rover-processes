import math

from robocluster import Device

from roverutil import getnetwork

Autopilot = Device('Autopilot', 'rover', network=getnetwork())

LOOP_PERIOD = 0.1
MIN_WHEEL_RPM = 1000
MAX_SPEED = 2 # m/s
BEARING_THRESH = 5 # degrees
GPS_DISTANCE_THRESH = 5 # meters

###### Initialization ########

Autopilot.storage.state = None
Autopilot.storage.enabled = False
Autopilot.storage.waypoints = []


######## State Machine ######
async def waiting():
    if Autopilot.storage.enabled:
        Autopilot.storage.waypoints = await Autopilot.request( 'Navigation', 'waypoints')
        print("Waypoints: ", Autopilot.storage.waypoints)
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
            a = bearing - heading
            a = (a+180)%360 - 180 # find smallest angle difference
            if abs(a) > BEARING_THRESH:
                if a >= 0: # Turn right
                    await Autopilot.send('DriveSystem', 'RotateRight', MAX_SPEED/60)
                else: # Turn left
                    await Autopilot.send('DriveSystem', 'RotateLeft', MAX_SPEED/60)
                return
            distance = await Autopilot.request('Navigation', 'distance', position, waypoints[0])
            print("distance from {} to {} = {}".format(position, waypoints[0], distance))
            if distance > GPS_DISTANCE_THRESH:
                await Autopilot.send('DriveSystem', 'DriveForward', MAX_SPEED)
            else:
                print('!!!!!!!HERE!!!!!!!!!!')
                # We are close enough TODO: search for ball
                Autopilot.storage.enabled = False
                await Autopilot.publish("Autopilot", False)  # update WebUI
                await Autopilot.publish("TargetReached", True)
                Autopilot.storage.state = waiting
    else:
        Autopilot.storage.state = waiting


async def search_for_ball():
    pass

async def drive_to_ball():
    pass


Autopilot.storage.state = waiting
@Autopilot.every(LOOP_PERIOD)
async def state_machine():
    print(Autopilot.storage.state.__name__)
    await Autopilot.storage.state()


######## Callbacks ###########

@Autopilot.on('*/Autopilot')
async def enable_autopilot(event, data):
    print('Setting Autopilot to {}'.format(data))
    Autopilot.storage.enabled = data

Autopilot.start()
Autopilot.wait()
