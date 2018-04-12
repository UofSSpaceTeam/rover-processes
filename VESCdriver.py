import time
import threading

import serial
from serial.tools import list_ports
import pyvesc


class ReaderThread(threading.Thread):
    def __init__(self, driver, callback):
        super().__init__()
        self.driver = driver
        self.exit = False
        self.callback = callback

    def run(self):
        while not self.exit:
            vesc_msg = self.driver.read()
            self.callback(vesc_msg)


class VESCDriver:
    '''Provides a simple pyvesc and Serial wrapper for VESC serial devices.'''

    def __init__(self, usbpath, baudrate=115200, read_callback=None):
        '''Create a new device.
        Args:
            usbpath (str): The path or comport for the serial device
            buadrate (int): The baudrate to communicate at. defaults to 115200.
            read_callback (function): If specified, a background thread will continually read from the device and call read_callback, passing in a pyvesc VESC message.
        '''
        self.usbpath = usbpath
        self.baudrate = 115200
        self.ser = serial.Serial(usbpath, baudrate=baudrate)
        if read_callback is not None:
            self.reader_thread = ReaderThread(self, read_callback)
            self.reader_thread.start()
        else:
            self.reader_thread = None


    def write(self, vesc_message):
        '''Send a pyvesc VESC message to the device'''
        b = pyvesc.encode(vesc_message)
        self.ser.write(b)

    def read(self):
        '''Read a pyvesc VESC message from the device'''
        # from Roveberrypy
        to_int = lambda b: int.from_bytes(b, byteorder='big')
        head = self.ser.read()
        # magic VESC header must be 2 or 3
        if not to_int(head) == 2 or to_int(head) == 3:
                return (None, None)
        length = self.ser.read(to_int(head) - 1)
        packet = head + length + self.ser.read(to_int(length) + 4)
        return pyvesc.decode(packet)[0]

    def stop(self):
        '''If a reader thread is running, shut it down.'''
        if self.reader_thread:
            self.reader_thread.exit = True
            self.reader_thread.join(0.1)

def test_driver():
    def callback(vesc_message):
        print(vesc_message)
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
