import math

from robocluster import Device

Autopilot = Device('Autopilot', 'rover')

LOOP_PERIOD = 0.1
MIN_WHEEL_RPM = 1000
BEARING_THRESH = 5 # degrees
GPS_DISTANCE_THRESH = 10 # meters

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
        await Autopilot.publish('wheelLF', {'SetRPM':0})
        await Autopilot.publish('wheelRF', {'SetRPM':0})

async def drive_to_target():
    if Autopilot.storage.enabled:
        position = await Autopilot.request('Navigation', 'RoverPosition')
        if position == [0,0]:
            return # initialization state
        waypoints = Autopilot.storage.waypoints
        if len(Autopilot.storage.waypoints) > 0:
            # Make sure we're pointed in the right direction
            bearing = await Autopilot.request('Navigation', 'bearing', position, waypoints[0])
            heading = await Autopilot.request('Navigation', 'RoverHeading')
            if abs(bearing - heading) > BEARING_THRESH: # TODO, math is wrong
                # TODO: Rotate to position
                pass
            distance = await Autopilot.request('Navigation', 'distance', position, waypoints[0])
            print("distance from {} to {} = {}".format(position, waypoints[0], distance))
            if distance > GPS_DISTANCE_THRESH:
                #TODO talk to DriveProcess instead
                await Autopilot.publish('wheelLF', {'SetRPM':MIN_WHEEL_RPM})
                await Autopilot.publish('wheelRF', {'SetRPM':MIN_WHEEL_RPM})
                pass
            else:
                print('!!!!!!!HERE!!!!!!!!!!')
                # We are close enough TODO: search for ball
                Autopilot.storage.enabled = False
                await Autopilot.publish("Autopilot", False)  # update WebUI
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
@Autopilot.on('*/roverHeading')
def update_heading(event, data):
    Autopilot.storage.heading = data

@Autopilot.on('*/GPSPosition')
def update_position(event, data):
    Autopilot.storage.position = data

@Autopilot.on('*/Autopilot')
def enable_autopilot(event, data):
    Autopilot.storage.enabled = data



Autopilot.start()
Autopilot.wait()
