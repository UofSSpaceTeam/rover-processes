from math import *
import random
import math
from statistics import mean
import sys

from robocluster import Device

from libraries.GPS.Piksi import Piksi

## Global Variables
LOOP_PERIOD = 0.3 # How often we pusblish positions

MSG_POS_LLH = 0x0201 # reports the absolute geodetic coordinate of the rover
MSG_VEL_NED = 0x0205 # velocity in north, east, down coordinates

NUM_SAMPLES = 10
SAMPLE_RATE = LOOP_PERIOD/NUM_SAMPLES # seconds in between samples

SERIAL = '/dev/ttyUSB0'
BAUD = 1000000

# TODO: We can't send GPSPosition over the network with JSON,
# we should move this somewhere else.
class GPSPosition:
    # Earth's radius in metres
    RADIUS = 6371008.8

    def __init__(self, lat, lon, mode=0):
        # lat an lon are assumed to be in radians
        self.lat = lat
        self.lon = lon
        self.mode = mode # 0=SPP, 1=Float RTK, 2=Fixed RTK

    def distance(self, them):
        ''' Returns the distance to another GPSPositions on earth'''
        hav = lambda z: (1 - cos(z)) / 2   # haversine
        ahav = lambda z: 2 * asin(sqrt(z)) # inverse haversine

        d_lat = them.lat - self.lat
        d_lon = them.lon - self.lon

        z = (hav(d_lat) + cos(self.lat) * cos(them.lat) * hav(d_lon))

        return GPSPosition.RADIUS * ahav(z)

    def bearing(self, them):
        ''' Returns the bearing to another GPSPositions on earth'''
        d_lat = them.lat - self.lat
        d_lon = them.lon - self.lon

        y = sin(d_lon) * cos(them.lat)
        x = (cos(self.lat) * sin(them.lat)
                        - sin(self.lat) * cos(them.lat) * cos(d_lat))

        return math.degrees(atan2(y, x))

    def gpsPosition(self, bearing, distance):
        r_lat = math.radians(self.lat)
        r_lon = math.radians(self.lon)
        target_lat = asin(sin(r_lat)*cos(distance/GPSPosition.RADIUS) +cos(r_lat)*sin(distance/GPSPosition.RADIUS)*cos(math.radians(bearing)))

        target_lon = r_lon + atan2(sin(bearing)*sin(distance/GPSPosition.RADIUS)*cos(r_lat),cos(distance/GPSPosition.RADIUS)-sin(r_lat)*sin(target_lat))
        return GPSPosition(math.degrees(target_lat), math.degrees(target_lon))


GPSdevice = Device('GPSdevice', 'rover')

@GPSdevice.every('1s')
async def dummy():
    await GPSdevice.publish('singlePointGPS', [51.0342432+(random.random()), 110.432342432+(random.random())])

# @GPSdevice.every(LOOP_PERIOD)
async def every():
    with Piksi(SERIAL, BAUD) as piksi:
        connected = piksi.connected()
        if not connected:
            print("Rover piksi is not connected properly")
        else:
            # print("Rover piksi is connected")
            lats = []
            longs = []
            for i in range(NUM_SAMPLES):
                # Take average of multiple samples.
                # Actually doesn't perform that well...
                msg = piksi.poll(MSG_POS_LLH)
                if msg is not None:
                    lats.append(msg.lat)
                    longs.append(msg.lon)
                await GPSdevice.sleep(SAMPLE_RATE)
            if len(lats) > 1:
                await GPSdevice.publish('singlePointGPS',
                        {'lat': mean(lats), 'lon':mean(longs)})
            else:
                print("Failed to take GPS averege", "WARNING")
            msg = piksi.poll(MSG_VEL_NED)
            if msg is not None:
                await GPSdevice.publish("GPSVelocity", [msg.n/1000, msg.e/1000])

if len(sys.argv) == 2:
    SERIAL = sys.argv[2]
else:
    print('Usage: GPSdriver.py <path-to-piksi>')
    print('Using default /dev/ttyUSB0')
GPSdevice.run()
