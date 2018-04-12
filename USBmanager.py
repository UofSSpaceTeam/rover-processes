import time
import asyncio

import serial.tools.list_ports
from pyvesc import ReqSubscription
from robocluster import Device

from VESCdriver import VESCDriver


manager = Device('USBManager', 'rover')
manager.storage.drivers = {}
manager.storage.sub_map = {}

def handle_vesc_message(vesc_message, path):
    if path in manager.storage.sub_map:
        sub = manager.storage.sub_map[path]
        @manager.task
        async def pub_message():
            print(sub, vesc_message)
            # TODO, translate VESC message into dict
            await manager.publish(sub, vesc_message.string)

@manager.task
def get_subscribers():
    PortList = serial.tools.list_ports.comports()
    for port in PortList:
        if port.device == '/dev/ttyS0':
            continue
        print(port.device)
        driver = VESCDriver(port.device)
        driver.write(ReqSubscription('t')) # blocking
        while True:
            msg = driver.read() # blocking
            if isinstance(msg, ReqSubscription):
                driver.start_reader(handle_vesc_message)
                manager.storage.drivers[msg.subscription] = driver
                manager.storage.sub_map[port.device] = msg.subscription
                break

@manager.every('1s')
def print_devices():
    print(manager.storage.drivers)

try:
    manager.run()
except KeyboardInterrupt:
    for i in manager.storage.drivers:
        manager.storage.drivers[i].stop()
