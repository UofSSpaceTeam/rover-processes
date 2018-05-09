import time
import asyncio
from concurrent.futures import ThreadPoolExecutor

import serial.tools.list_ports
from robocluster import Device
import pyvesc

from VESCdriver import VESCDriver

IGNORE_LIST = ['/dev/ttyS0', '/dev/ttyAMA0']


manager = Device('USBManager', 'rover')
manager.storage.drivers = {}
manager.storage.sub_map = {}

def vesc_to_dict(vesc_message):
    '''Takes a pyvesc vesc message and makes a dictionary of the form:
       {'ExampleSendMessage':{'string':<value of string>}}

       The value of the top level dictionary is the __dict__ attribute of the message and contains all the fields of the message.
    '''
    d = {
        vesc_message.__class__.__name__:vesc_message.__dict__
    }
    return d

def dict_to_vesc(d):
    '''Sort of the opposite of vesc_to_dict, but takes a different dict format:

       {<VESCMessage name>:[List, of, constructor, parameters]}

    '''
    name = list(d)[0]
    if not isinstance(d[name], list):
        params = [d[name]]
    else:
        params = d[name]
    msg = getattr(pyvesc, name)(*params)
    return msg


def handle_vesc_message(vesc_message, path):
    if path in manager.storage.sub_map:
        sub = manager.storage.sub_map[path]
        @manager.task
        async def pub_message():
            await manager.publish(sub, vesc_to_dict(vesc_message))

def get_subscribers():
    PortList = serial.tools.list_ports.comports()
    manager.executor = ThreadPoolExecutor(max_workers=len(PortList))
    for port in PortList:
        if port.device in IGNORE_LIST:
            continue
        print('Found {}'.format(port.device))
        driver = VESCDriver(port.device)
        driver.write(pyvesc.ReqSubscription('t')) # blocking
        while True:
            msg = driver.read() # blocking
            if isinstance(msg, pyvesc.ReqSubscription):
                driver.start_reader(handle_vesc_message)
                manager.storage.drivers[msg.subscription] = driver
                manager.storage.sub_map[port.device] = msg.subscription
                print('*/'+msg.subscription)
                @manager.on('*/'+msg.subscription)
                async def _write_to_device(event, data):
                    if 'USBManager' in event:
                        # So we don't listen to ourself TODO: use negation in globbing instead?
                        return
                    subscription = event.split('/')[-1]
                    dvr = manager.storage.drivers[subscription]
                    vesc_message = dict_to_vesc(data)
                    await manager.loop.run_in_executor(
                            manager.executor, dvr.write, vesc_message)

                break

@manager.every('1s')
def print_devices():
    print("Devices: ", [key for key in manager.storage.drivers.keys()])


get_subscribers()
manager.start()
manager.wait()
