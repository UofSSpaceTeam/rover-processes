from robocluster import SerialDriver, Device
import serial.tools.list_ports

PortList = serial.tools.list_ports.comports()

ignore_list = ['/dev/ttyS0', '/dev/ttyUSB0']

ports = {}
for port in PortList:
    if port.device in ignore_list:
        continue
    print(port.device)
    ports[port.device] = SerialDriver(port.device,'rover')
    ports[port.device].start()

try:
    for i in ports:
        ports[i].wait()
except KeyboardInterrupt:
    for i in ports:
        ports[i].stop()
