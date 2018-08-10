from robocluster import Device

ScienceDevice = Device('ScienceDevice', 'rover')
ScienceDevice.storage.zero_switch = 0


@ScienceDevice.on('*/move_carousel')
async def move_carousel(event, data):
    await ScienceDevice.publish('ScienceArduino', {'move_carousel': int(data)})

@ScienceDevice.on('*/run_emitter')
async def move_carousel(event, data):
    await ScienceDevice.publish('ScienceArduino', {'run_emitter': int(data)})

@ScienceDevice.on('*/Stage1')
async def stage_one(event, data):
    await ScienceDevice.publish('ScienceArduino', {'move_carousel': 137+10})
    await ScienceDevice.sleep(5)
    await ScienceDevice.publish('ScienceArduino', {'run_emitter': 1})
    await ScienceDevice.sleep(1)
    await ScienceDevice.publish('ScienceArduino', {'take_reading': 0})
    await ScienceDevice.sleep(0.1)

@ScienceDevice.on('*/Stage2')
async def stage_two(event, data):
    await ScienceDevice.publish('ScienceArduino', {'move_carousel': 68+10})
    await ScienceDevice.sleep(5)
    for i in range(150):
        await ScienceDevice.publish('ScienceArduino', {'take_reading': 0})
        await ScienceDevice.sleep(0.1)
    await ScienceDevice.sleep(0.1)
    await ScienceDevice.publish('ScienceArduino', {'run_emitter': 0})
    await ScienceDevice.publish('Spectrometer_samples', ScienceDevice.storage.samples)
    await ScienceDevice.publish('ScienceArduino', {'move_carousel': -207-10})

@ScienceDevice.on('*/spectrometer_data')
def save_sample(event, data):
    ScienceDevice.storage.samples.append(data)

# @ScienceDevice.every('100ms')
# async def poll_detector():
#     await ScienceDevice.publish('ScienceArduino', {'take_reading': 0})

@ScienceDevice.on('*/science_limit_switches')
async def switches(event, data):
    # log.debug('Updating limit switches {}'.format(data))
    ScienceDevice.storage.zero_switch = data[2]

ScienceDevice.storage.samples = []

ScienceDevice.start()
ScienceDevice.wait()
