import time
import threading
import json

import serial

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
        b = json.encode(msg)
        print(b)
        self.ser.write(b)

    def read(self):
        '''Read a json message from the device'''
        # from Roveberrypy
        size = self.ser.read(1) # this is assuming a message is less than 256 bytes
        length = int.from_bytes(size, byteorder='big')
        packet = self.ser.read(length)
        return json.loads(packet.decode('utf8'))

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

def test_driver():
    def callback(msg):
        print(msg)
    driver = VESCDriver('/dev/ttyACM0', read_callback=callback)
    msg = pyvesc.BlinkLed(0)
    try:
        driver.write(msg)
        time.sleep(1)
        msg = pyvesc.BlinkLed(1)
        driver.write(msg)
        time.sleep(1)
        msg = pyvesc.BlinkLed(0)
        driver.write(msg)
        time.sleep(1)
        print('done')
    except KeyboardInterrupt:
        pass
    driver.stop()


if __name__ == '__main__':
    test_driver()
