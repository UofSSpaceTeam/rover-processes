from robocluster import Device

device = Device('requestor', 'rover')

def get_pos(resp):
    '''Gets the rotor position from a GetRotorPosition request'''
    try:
        position = resp['GetRotorPosition']['rotor_pos']
        return position
    except KeyError:
        return None

@device.every('100ms')
async def request_pos():
    resp = await device.request('USBManager', 'armWrist', {'GetRotorPosition':42})
    pos = get_pos(resp)
    print(pos)

device.start()
device.wait()
