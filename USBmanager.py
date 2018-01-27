from robocluster import SerialDriver, Device
import serial.tools.list_ports

myDevice = Device('myDevice', 'rover')
@myDevice.on('*/sensor1')
async def funtion(event_name, data):
    print(event_name, data)

PortList = serial.tools.list_ports.comports()

ports = {}
for port in PortList:
    print(port.device)
    ports[port.device] = SerialDriver(port.device,'rover')
    ports[port.device].start()

try:
    myDevice.start()
    for i in ports:
        ports[i].wait()
    myDevice.wait()
except KeyboardInterrupt:
    for i in ports:
        ports[i].stop()
    myDevice.stop()
