from robocluster import SerialDriver, Device
from pyvesc import SetRPM

driver = SerialDriver('/dev/ttyACM0', 'rover', encoding='vesc')
drive_device = Device('drive', 'rover')

@driver.every('1s')
async def print_name():
    print(driver.name)

@driver.every('100ms')
async def spin():
    await driver.write(SetRPM(int(1500)))

@drive_device.every('100ms')
async def rpm():
    await drive_device.send('wheelLM', 'SetRPM', 1500)

driver.run()
