from robocluster import Device
import config

log = config.logger
CONTROLLER_NUM = config.front_end_loader_controller


FELoaderDevice = Device('FELoaderDevice', 'rover', network=config.network)

# Initialize setup variables
# ...

@FELoaderDevice.on('*/controller{}/joystick1'.format(CONTROLLER_NUM))
async def joystick1_callback(joystick1, data):
    """ Handles the <...> linear actuator for manual control
            A Joystick message contains:
            [x axis (float -1:1), y axis (float -1:1)]
    """
    y_axis = data[1]
    if y_axis is None:
        return
    
@FELoaderDevice.on('*/controller{}/joystick2'.format(CONTROLLER_NUM))
async def joystick2_callback(joystick2, data):
    """ Handles the <...> linear actuator for manual control
            A Joystick message contains:
            [x axis (float -1:1), y axis (float -1:1)]
    """
    y_axis = data[1]
    if y_axis is None:
        return
    await FELoaderDevice.publish('

    
### More motors // DoF ?
### Duty Cycle?
