import time
import os

import pygame
from robocluster import Device

import config

LOOP_PERIOD = 0.1 # seconds

os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
pygame.joystick.init()

JoystickDevice = Device('JoystickDevice', 'rover', network=config.network)

JoystickDevice.storage.joystick1 = pygame.joystick.Joystick(0)
JoystickDevice.storage.joystick1.init()

@JoystickDevice.every(LOOP_PERIOD)
async def read_joystick():
    pygame.event.get()
    left = [JoystickDevice.storage.joystick1.get_axis(0), JoystickDevice.storage.joystick1.get_axis(1)]
    right = [JoystickDevice.storage.joystick1.get_axis(3), JoystickDevice.storage.joystick1.get_axis(4)]
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
