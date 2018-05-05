"""
Keeps track of the Rover's position and environment.
Provides a way for other processes to query this information of the
environment.
"""

import math

from robocluster import Device

from libraries.GPS.GPSPosition import GPSPosition


class Rover:

    def __init__(self):
        self.position = GPSPosition(0, 0)
        self.heading = 0
        self.wheel_speed = [0, 0]


NavDevice = Device('Navigation', 'rover')

NavDevice.storage.rover = Rover()
NavDevice.storage.waypoints = [(52.132866, -106.628012)]

@NavDevice.on('*/FilteredGPS')
@NavDevice.on('*/GPSPosition')
# @NavDevice.on('*/singlePointGPS')
def udpate_position(event, data):
    NavDevice.storage.rover.position = GPSPosition(*data)

@NavDevice.on('*/roverHeading')
def update_heading(event, data):
    NavDevice.storage.rover.heading = data

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


NavDevice.start()
NavDevice.wait()
