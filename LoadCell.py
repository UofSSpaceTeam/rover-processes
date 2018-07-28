from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from libraries.PhidgetHelperFunctions import *

from robocluster import Device
import config
log = config.getLogger()

DATA_INTERVAL = 100
SERIAL_NUMBER = 532721
CHANNEL = 2

LoadDevice = Device('LoadCell', 'rover', network=config.network)

def on_Voltage(ph, voltageRatio):
    log.info(voltageRatio)
    @LoadDevice.task
    async def send_reading():
        await LoadDevice.publish('LoadCell', voltageRatio)


def onErrorHandler(ph, errorCode, errorString):
    log.error(errorString)

def onAttachHandler(ph):
    ph.setDataInterval(DATA_INTERVAL)
    ph.setVoltageRatioChangeTrigger(0.0)
    if(ph.getChannelSubclass() == ChannelSubclass.PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT):
        # print("\tSetting VoltageRatio SensorType")
        ph.setSensorType(VoltageRatioSensorType.SENSOR_TYPE_VOLTAGERATIO)

channel = VoltageRatioInput()
channel.setDeviceSerialNumber(SERIAL_NUMBER)
channel.setChannel(CHANNEL)
channel.setIsHubPortDevice(False)
channel.setHubPort(-1)

channel.setOnAttachHandler(onAttachHandler)
channel.setOnErrorHandler(onErrorHandler)
channel.setOnVoltageRatioChangeHandler(on_Voltage)
channel.openWaitForAttachment(5000)

LoadDevice.start()
LoadDevice.wait()
