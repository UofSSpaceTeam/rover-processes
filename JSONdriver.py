import time
import threading
import json

import serial

import config
log = config.getLogger()

class ReaderThread(threading.Thread):
    def __init__(self, driver, callback):
        super().__init__()
        self.driver = driver
        self.exit = False
        self.callback = callback
        self.daemon = True

    def run(self):
        while not self.exit:
            msg = self.driver.read()
            if msg is not None:
                self.callback(msg, self.driver.usbpath)

class JSONDriver:
    '''Provides a simple Serial wrapper for JSON serial devices.'''

    def __init__(self, usbpath, baudrate=115200, read_callback=None):
        '''Create a new device.
        Args:
            usbpath (str): The path or comport for the serial device
            buadrate (int): The baudrate to communicate at. defaults to 115200.
            read_callback (function): If specified, a background thread will
            continually read from the device and call read_callback,
            passing in a JSON message and the usbpath it came from.
        '''
        self.usbpath = usbpath
        self.baudrate = 115200
        self.ser = serial.Serial(usbpath, baudrate=baudrate)
        self.start_reader(read_callback)


    def write(self, msg):
        '''Send a json message to the device'''
        b = json.dumps(msg)
        log.debug('Writing {} bytes {}'.format(len(b), b))
        # self.ser.write(len(b))
        self.ser.write(bytes([len(b)])+b.encode())

    def read(self):
        '''Read a json message from the device'''
        # from Roveberrypy
        try:
            size = self.ser.read(1) # this is assuming a message is less than 256 bytes
            length = int.from_bytes(size, byteorder='big')
            packet = self.ser.read(length)
            log.debug('Reading {} bytes {}'.format(length, packet))
            return json.loads(packet.decode('utf8'))
        except json.decoder.JSONDecodeError:
            return None

    def start_reader(self, callback):
        ''' Starts a background reader thread.
        Args:
            callback (function): If specified, a background thread
            will continually read from the device and call read_callback,
            passing in a pyvesc VESC message and the usbpath it came from.
        '''
        if callback is not None:
            self.reader_thread = ReaderThread(self, callback)
            self.reader_thread.start()
        else:
            self.reader_thread = None


    def stop(self):
        '''If a reader thread is running, shut it down.'''
        if self.reader_thread:
            self.reader_thread.exit = True
            self.reader_thread.join(0.1)
