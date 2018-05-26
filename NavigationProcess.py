"""
Keeps track of the Rover's position and environment.
Provides a way for other processes to query this information of the
environment.
"""

import math

from robocluster import Device

from libraries.GPS.GPSPosition import GPSPosition

from roverutil import getnetwork


class Rover:

    def __init__(self):
        self.position = GPSPosition(0, 0)
        self.heading = 0
        self.wheel_speed = [0, 0]
        self.pitch = 0
        self.roll = 0


NavDevice = Device('Navigation', 'rover', getnetwork())

NavDevice.storage.rover = Rover()
# NavDevice.storage.waypoints = [(52.132866, -106.628012)]
NavDevice.storage.waypoints = [(52.132774, -106.627528)]
NavDevice.storage.turn_direction = None

@NavDevice.on('*/FilteredGPS')
@NavDevice.on('*/GPSPosition')
# @NavDevice.on('*/singlePointGPS')
def udpate_position(event, data):
    NavDevice.storage.rover.position = GPSPosition(*data)

@NavDevice.on('*/roverHeading')
def update_heading(event, data):
    NavDevice.storage.rover.heading = data

@NavDevice.on('*/CompassDataMessage')
def compas_callback(event, data):
    NavDevice.storage.rover.heading = data['heading']+10.26667 #magnetic declination offset 10.58333 for Hanksville
    NavDevice.storage.rover.pitch = data['pitch']
    NavDevice.storage.rover.roll = data['roll']

@NavDevice.on('*/AccelerometerMessage')
async def accelerometer_callback(event, data):
    await NavDevice.publish('Acceleration', [data['x'], -data['y']])

@NavDevice.on('*/DirectionToTurn')
async def update_turn_dircetion(event, data):
    NavDevice.storage.turn_direction = data

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


NavDevice.start()
NavDevice.wait()
