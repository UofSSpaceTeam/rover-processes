from robocluster import Device

ScienceDevice = Device('ScienceDevice', 'rover')


@ScienceDevice.on('*/move_carousel')
async def move_carousel(event, data):
    await ScienceDevice.publish('ScienceArduino', {'move_carousel': int(data)})

@ScienceDevice.on('*/run_emitter')
async def move_carousel(event, data):
    await ScienceDevice.publish('ScienceArduino', {'run_emitter': int(data)})

@ScienceDevice.on('*/Stage1')
async def stage_one(event, data):
    await ScienceDevice.publish('ScienceArduino', {'move_carousel': 137})
    await ScienceDevice.sleep(5)
    await ScienceDevice.publish('ScienceArduino', {'run_emitter': 1})
    await ScienceDevice.sleep(1)
    await ScienceDevice.publish('ScienceArduino', {'take_reading': 0})
    await ScienceDevice.sleep(0.1)

@ScienceDevice.on('*/Stage2')
async def stage_two(event, data):
    await ScienceDevice.publish('ScienceArduino', {'move_carousel': 68})
    await ScienceDevice.sleep(5)
    for i in range(10):
        await ScienceDevice.publish('ScienceArduino', {'take_reading': 0})
        await ScienceDevice.sleep(0.1)
    await ScienceDevice.sleep(0.1)
    await ScienceDevice.publish('ScienceArduino', {'run_emitter': 0})
    await ScienceDevice.publish('Spectrometer_samples', ScienceDevice.storage.samples)
    await ScienceDevice.publish('ScienceArduino', {'move_carousel': -207})

@ScienceDevice.on('*/spectrometer_data')
def save_sample(event, data):
    ScienceDevice.storage.samples.append(data)

@ScienceDevice.every('100ms')
async def poll_detector():
    await ScienceDevice.publish('ScienceArduino', {'take_reading': 0})


ScienceDevice.storage.samples = []

ScienceDevice.start()
ScienceDevice.wait()
