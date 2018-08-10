"""
Keeps track of the Rover's position and environment.
Provides a way for other processes to query this information of the
environment.
"""

import math

from robocluster import Device

from libraries.GPS.GPSPosition import GPSPosition

import config
log = config.getLogger()


class Rover:

    def __init__(self):
        self.position = GPSPosition(0, 0)
        self.heading = 0
        self.wheel_speed = [0, 0]
        self.pitch = 0
        self.roll = 0


NavDevice = Device('Navigation', 'rover', network=config.network)

NavDevice.storage.rover = Rover()
# NavDevice.storage.waypoints = [(52.132866, -106.628012)]
NavDevice.storage.turn_direction = None
NavDevice.storage.waypoints = []
NavDevice.storage.yagipower = [None]*36

NavDevice.storage.ball = None

@NavDevice.on('*/FilteredGPS')
@NavDevice.on('*/GPSPosition')
# @NavDevice.on('*/singlePointGPS')
def udpate_position(event, data):
    NavDevice.storage.rover.position = GPSPosition(*data)

@NavDevice.on('*/YagiPower')
async def update_rdf_power(event, data):
    heading = NavDevice.storage.rover.heading
    angle = int(math.floor(heading/10))
    previous_val = NavDevice.storage.yagipower[angle]
    if previous_val is None or previous_val <= data:
        NavDevice.storage.yagipower[angle] = data
    await NavDevice.publish('RDF_readings', NavDevice.storage.yagipower)

@NavDevice.on('*/roverHeading')
def update_heading(event, data):
    NavDevice.storage.rover.heading = data

@NavDevice.on('*/CompassDataMessage')
def compas_callback(event, data):
    NavDevice.storage.rover.heading = data['heading'] #magnetic declination offset 10.58333 for Hanksville 10.26667 for SK
    NavDevice.storage.rover.pitch = data['pitch']
    NavDevice.storage.rover.roll = data['roll']

@NavDevice.on('*/AccelerometerMessage')
async def accelerometer_callback(event, data):
    await NavDevice.publish('Acceleration', [data['x'], -data['y']])

@NavDevice.on('*/DirectionToTurn')
async def update_turn_dircetion(event, data):
    NavDevice.storage.turn_direction = data

@NavDevice.on('*/sendWaypoints')
def update_waypoints(event, data):
    NavDevice.storage.waypoints = data
    log.debug('Waypoints: {}'.format(data))

@NavDevice.on('*/OpenMV')
def update_ball_info(event, data):
    if data['x'] == -1 and data['y'] == -1:
        NavDevice.storage.ball = None
    else:
        ball_x = data['x'] - data['sensorWidth']/2 # distance from center of img
        ball_y = data['y'] - data['sensorHeight']/2
        NavDevice.storage.ball = {'x':ball_x, 'y':ball_y, 'size':data['size']}
    log.debug(NavDevice.storage.ball)

@NavDevice.on_request('RoverPosition')
def return_position():
    pos = NavDevice.storage.rover.position
    return [pos.lat, pos.lon]


@NavDevice.on_request('RoverHeading')
def return_heading():
    return NavDevice.storage.rover.heading

@NavDevice.on_request('waypoints')
def return_waypoint():
    return NavDevice.storage.waypoints

@NavDevice.on_request('bearing')
def calculate_bearing(start, dest):
    p0 = GPSPosition(*start)
    p1 = GPSPosition(*dest)
    return p0.bearing(p1)

@NavDevice.on_request('distance')
def cacluate_distance(start, dest):
    p0 = GPSPosition(*start)
    p1 = GPSPosition(*dest)
    return p0.distance(p1)

@NavDevice.on_request('DirectionToTurn')
def return_turn_dir():
    return NavDevice.storage.turn_direction

@NavDevice.on_request('BallPosition')
def return_ball_pos():
    return NavDevice.storage.ball


NavDevice.start()
NavDevice.wait()
