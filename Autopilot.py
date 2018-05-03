
from robocluster import Device

Autopilot = Device('Autopilot', 'rover')

LOOP_PERIOD = 0.1
MIN_WHEEL_RPM = 20000

###### Initialization ########
Autopilot.storage.target = None
Autopilot.storage.heading = 0
Autopilot.storage.position = None

Autopilot.storage.state = None
Autopilot.storage.enabled = False
Autopilot.storage.waypoints = [(1,1)]


######## State Machine ######
async def waiting():
    if Autopilot.storage.enabled and\
            len(Autopilot.storage.waypoints) > 0:
        Autopilot.storage.state = drive_to_target

async def drive_to_target():
    if Autopilot.storage.enabled:
        await Autopilot.publish('wheelLF', {'SetRPM':MIN_WHEEL_RPM})
        await Autopilot.publish('wheelRF', {'SetRPM':MIN_WHEEL_RPM})
        # TODO: Actually drive to target
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
