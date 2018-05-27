from robocluster import Device


ScienceDevice = Device('ScieneModue', 'rover')

@ScienceDevice.task
async def start_sci():
    await ScienceDevice.sleep(2)
    await ScienceDevice.publish('ScienceArduino', {'enable_science':1})
    print('Sent science to start')


ScienceDevice.start()
ScienceDevice.wait()
