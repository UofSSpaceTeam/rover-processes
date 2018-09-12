"""
USBmanager
==========
The USB manager acts as a translation layer between
USB serial devices and the rest of robocluster.
There are some hacks to support devices that
talk VESC or JSON.

The reason for having this manager that everything talks through
is to decouple the logic of the main processes from the communication
with embedded devices. The USB manager also handles identities of devices
as when you plug them in they show up as 'COM1', 'COM2', '/dev/ttyACM0'
which is undescriptive. Serial devices connected to the USB manager
must respond to an identity request message (referred to as its "subscription",
a carry over from Roveberrypy).
"""
import time
import asyncio
from concurrent.futures import ThreadPoolExecutor
import json

import serial.tools.list_ports
from robocluster import Device
import pyvesc
import serial

from VESCdriver import VESCDriver
from JSONdriver import JSONDriver
import config
log = config.getLogger()

# USB manager will ignore these devices as they break things
# or use a different protocol that will be used by a process
# directly.
# ttyUSB0 is the Piksi GPS unit.
# ttyAMA0 is the serial pins on the Raspberry pi.
# ttyS0 is a virtual serial console on Linux that is always present.
IGNORE_LIST = ['/dev/ttyS0', '/dev/ttyAMA0', '/dev/ttyUSB0']


manager = Device('USBManager', 'rover', network=config.network)
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
    """
    Take a VESC message from the usb device and publish it over robocluster.
    Args:
        vesc_message: A VESCMessage object from pyvesc.
        path (str): The path (Posix) or com port (windows) to the serial device.
    """
    if path in manager.storage.sub_map:
        sub = manager.storage.sub_map[path]
        @manager.task
        async def pub_message():
            msg = vesc_to_dict(vesc_message)
            key = list(msg.keys())[0]
            await manager.publish(sub + '/' + key, msg[key])

def handle_json_message(json_message, path):
    """
    Take a JSON message from the usb device and publish it over robocluster.
    Args:
        json_message: A python object created from json.load. Assumed to be a dict.
        path (str): The path (Posix) or com port (windows) to the serial device.
    """
    if path in manager.storage.sub_map:
        sub = manager.storage.sub_map[path]
        @manager.task
        async def pub_message():
            if type(json_message) == dict:
                # TODO: support more than just a dict?
                for key in json_message.keys():
                    await manager.publish(sub+'/'+key, json_message[key])

def init_vesc_driver(port, ser, l):
    """
    Create a VESCDriver for the given port. Finishes reading the response to
    the identity request packet from the device.
    Args:
        port (serial.tools.list_ports.ListPortInfo): The serial port to create the driver for.
        ser: (serial.Serial): The Serial object created for the port
        l: The first byte of the received VESC packet, determines how many bytes the length
           field is. http://vedder.se/2015/10/communicating-with-the-vesc-using-uart/
    """
    to_int = lambda x: int.from_bytes(x, byteorder='big')
    length = ser.read(to_int(l) - 1)
    packet = l + length + ser.read(to_int(length) + 3)

    msg, _ = pyvesc.decode(packet)
    log.debug(msg)
    if isinstance(msg, pyvesc.ReqSubscription):
        log.debug('Creating VESC driver')
        ser.close()
        driver = VESCDriver(port.device)
        if msg.subscription not in ['armShoulder', 'armElbow', 'armWristPitch']: # hacks for position request
            driver.start_reader(handle_vesc_message)
        sub = msg.subscription
        manager.storage.drivers[sub] = driver
        manager.storage.sub_map[port.device] = sub
        # subscribe
        @manager.on('*/'+sub)
        async def _write_to_device(event, data):
            if 'USBManager' in event:
                # So we don't listen to ourself TODO: use negation in globbing instead?
                return
            subscription = event.split('/')[-1]
            dvr = manager.storage.drivers[subscription]
            vesc_message = dict_to_vesc(data)
            await manager.loop.run_in_executor(
                    manager.executor, dvr.write, vesc_message)

            # request handler
            @manager.on_request(msg.subscription)
            async def _request_from_vesc(vesc_msg):
                dvr = manager.storage.drivers[msg.subscription]
                vesc_msg = dict_to_vesc(vesc_msg)
                await manager.loop.run_in_executor(
                        manager.executor, dvr.write_request, vesc_msg)
                resp = await manager.loop.run_in_executor(
                        manager.executor, dvr.read)
                resp = vesc_to_dict(resp)
                return resp
        return True
    else:
        log.warning('Got bad subscription')
        return False

def init_json_driver(port, ser, l):
    """
    Create a JSONDriver for the given port. Finishes reading the response to
    the idendity request message.
    Args:
        port (serial.tools.list_ports.ListPortInfo): The serial port to create the driver for.
        ser: (serial.Serial): The Serial object created for the port
        l (byte): The length of the identity packet.
    """
    try:
        leng = int.from_bytes(l, byteorder='big')
        packet = ser.read(leng)
        log.debug('Received from {}: {}'.format(port, packet))
        msg = json.loads(packet.decode('utf8'))
        sub = msg['name']
    except KeyError:
        return False
    except json.decoder.JSONDecodeError:
        return False
    ser.close()
    driver = JSONDriver(port.device)
    driver.start_reader(handle_json_message)
    manager.storage.drivers[sub] = driver
    manager.storage.sub_map[port.device] = sub
    @manager.on('*/'+sub)
    async def _write_to_device(event, data):
        if 'USBManager' in event:
            # So we don't listen to ourself TODO: use negation in globbing instead?
            return
        subscription = event.split('/')[-1]
        dvr = manager.storage.drivers[subscription]
        await manager.loop.run_in_executor(
                manager.executor, dvr.write, data)
    return True

def get_subscribers():
    """
    Poll each serial device connected to the computer, and
    initialize the appropriate driver for it.
    """
    # TODO: make calls to each device asynchronous so we don't block
    # trying to read from an unresponsive device on start up.
    PortList = serial.tools.list_ports.comports()
    manager.executor = ThreadPoolExecutor(max_workers=len(PortList))
    for port in PortList:
        if port.device in IGNORE_LIST:
            continue
        log.info('Found {}'.format(port.device))
        ser = serial.Serial(port.device, baudrate=115200)
        looping = True
        while looping:
            b = pyvesc.encode(pyvesc.ReqSubscription('t'))
            ser.reset_input_buffer()
            ser.write(b)
            l = ser.read()
            if l == b'\x02' or l == b'\x03':
                # It's a VESC message!
                looping = not init_vesc_driver(port, ser, l)
            else:
                # It's a JSON message!
                looping = not init_json_driver(port, ser, l)


@manager.every('1s')
def print_devices():
    log.info("Devices: {}".format([key for key in manager.storage.drivers.keys()]))


get_subscribers()
manager.start()
manager.wait()
