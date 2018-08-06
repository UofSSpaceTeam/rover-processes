from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from libraries.PhidgetHelperFunctions import *

from robocluster import Device
import config
log = config.getLogger()

DATA_INTERVAL = 100
SERIAL_NUMBER = 532721

LoadDevice = Device('LoadCell', 'rover', network=config.network)

def on_Voltage(ph, voltageRatio):
    log.info(voltageRatio)
    if ph.channel == 0:
        weight = 8316008.31601*float(voltageRatio) - 585.072765073
    else:
        weight = 385728.061716*float(voltageRatio) + 95.2285438766
    @LoadDevice.task
    async def send_reading():
        await LoadDevice.publish('load_cell_weight', [ph.channel, weight])


def onErrorHandler(ph, errorCode, errorString):
    log.error(errorString)

def onAttachHandler(ph):
    ph.setDataInterval(DATA_INTERVAL)
    ph.setVoltageRatioChangeTrigger(0.0)
    if(ph.getChannelSubclass() == ChannelSubclass.PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT):
        # print("\tSetting VoltageRatio SensorType")
        ph.setSensorType(VoltageRatioSensorType.SENSOR_TYPE_VOLTAGERATIO)

channels = []
for i in range(2):
    channel = VoltageRatioInput()
    channel.setDeviceSerialNumber(SERIAL_NUMBER)
    channel.setChannel(i)
    channel.setIsHubPortDevice(False)
    channel.setHubPort(-1)

    channel.setOnAttachHandler(onAttachHandler)
    channel.setOnErrorHandler(onErrorHandler)
    channel.setOnVoltageRatioChangeHandler(on_Voltage)
    channel.openWaitForAttachment(5000)

    channels[i] = channel

LoadDevice.start()
LoadDevice.wait()
