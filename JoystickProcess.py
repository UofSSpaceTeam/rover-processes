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

''' different operating systems have different values associated with joysticks & buttons on 
an xbox360 controller, this function makes both os's work.

Note: if on windows, should run this file in command prompt rather than a linux distro because
i think the linux terminal doesnt have access to the same device info that command prompt does'''
def buttonAssign():
    if os.name == 'nt':
        rightHoriz = 4
        rightVert = 3
    else:
        rightHoriz = 3
        rightVert = 4
    return rightHoriz, rightVert

@JoystickDevice.every(LOOP_PERIOD)
async def every():
    # assigns os specific joystick assignments
    rightHor = buttonAssign()[0]
    rightVer = buttonAssign()[1]
    pygame.event.get()
    left = [JoystickDevice.storage.joystick1.get_axis(0), JoystickDevice.storage.joystick1.get_axis(1)]
    right = [JoystickDevice.storage.joystick1.get_axis(rightHor), JoystickDevice.storage.joystick1.get_axis(rightVer)]
    triggerL = JoystickDevice.storage.joystick1.get_axis(2)
    triggerR = JoystickDevice.storage.joystick1.get_axis(5)
    buttonA = JoystickDevice.storage.joystick1.get_button(0)
    buttonB = JoystickDevice.storage.joystick1.get_button(1)
    buttonX = JoystickDevice.storage.joystick1.get_button(2)
    buttonY = JoystickDevice.storage.joystick1.get_button(3)
    bumperL = JoystickDevice.storage.joystick1.get_button(4)
    bumperR = JoystickDevice.storage.joystick1.get_button(5)
    dpad = JoystickDevice.storage.joystick1.get_hat(0)

    await JoystickDevice.publish('joystick1', left)
    await JoystickDevice.publish('joystick2', right)
    await JoystickDevice.publish('triggerL', triggerL)
    await JoystickDevice.publish('triggerR', triggerR)

    await JoystickDevice.publish('buttonA', buttonA)
    await JoystickDevice.publish('buttonB', buttonB)
    await JoystickDevice.publish('buttonX', buttonX)
    await JoystickDevice.publish('buttonY', buttonY)

    await JoystickDevice.publish('bumperL', bumperL)
    await JoystickDevice.publish('bumperR', bumperR)

    await JoystickDevice.publish('dpad', dpad)


try:
    JoystickDevice.start()
    JoystickDevice.wait()
except KeyboardInterrupt:
    JoystickDevice.stop()
