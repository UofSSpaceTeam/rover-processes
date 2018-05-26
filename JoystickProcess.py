import time
import os
from roverutil import getnetwork

import pygame
from robocluster import Device

LOOP_PERIOD = 0.1 # seconds

os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
pygame.joystick.init()

JoystickDevice = Device('JoystickDevice', 'rover', network=getnetwork())

JoystickDevice.storage.joystick1 = pygame.joystick.Joystick(0)

JoystickDevice.storage.joystick1.init()

button_names = {'buttonA':0, 'buttonB':1, 'buttonX':2, 'buttonY':3,
        'bumperL':4, 'bumperR':5}
JoystickDevice.storage.last_state = {key:0 for key in button_names.keys()}

''' different operating systems have different values associated with joysticks & buttons on 
an xbox360 controller, this function makes both os's work.

Note: if on windows, should run this file in command prompt rather than a linux distro because
i think the linux terminal doesnt have access to the same device info that command prompt does'''
def buttonAssign():
    if os.name == 'nt':
        rightHoriz = 4
        rightVert = 3
        trigger = 2
    else:
        rightHoriz = 3
        rightVert = 4
        trigger = [2,5]
    return rightHoriz, rightVert, trigger


@JoystickDevice.every(LOOP_PERIOD)
async def every():
    # assigns os specific joystick assignments
    rightHor = buttonAssign()[0]
    rightVer = buttonAssign()[1]
    trigger = buttonAssign()[2]
    pygame.event.get()
    left = [JoystickDevice.storage.joystick1.get_axis(0), JoystickDevice.storage.joystick1.get_axis(1)]
    right = [JoystickDevice.storage.joystick1.get_axis(rightHor), JoystickDevice.storage.joystick1.get_axis(rightVer)]
    # windows treats two triggers as one value, so this code makes linux behave in the same way for the sake of consistency
    if os.name == 'nt':
        trig = JoystickDevice.storage.joystick1.get_axis(trigger)
    else:
        trig = 1 + JoystickDevice.storage.joystick1.get_axis(trigger[0]) + JoystickDevice.storage.joystick1.get_axis(trigger[1])



    await JoystickDevice.publish('joystick1', left)
    await JoystickDevice.publish('joystick2', right)
    await JoystickDevice.publish('trigger', trig)
    print('left: ' + str(left))
    print('right: ' + str(right))
    print('trig: ' + str(trig))

    for button in button_names.keys():
        button_val = JoystickDevice.storage.joystick1.get_button(button_names[button])
        last_value = JoystickDevice.storage.last_state[button]
        if button_val == 1 and last_value == 0:
            await JoystickDevice.publish(button+'_down', button_val)
        elif button_val == 0 and last_value == 1:
            await JoystickDevice.publish(button+'_up', button_val)
        await JoystickDevice.publish(button, button_val)
        JoystickDevice.storage.last_state[button] = button_val

    dpad = JoystickDevice.storage.joystick1.get_hat(0)
    await JoystickDevice.publish('dpad', dpad)



try:
    JoystickDevice.start()
    JoystickDevice.wait()
except KeyboardInterrupt:
    JoystickDevice.stop()
