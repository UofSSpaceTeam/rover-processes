import time
import os
import pygame
from robocluster import Device

LOOP_PERIOD = 0.1 # seconds

os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
pygame.joystick.init()

JoystickDevice = Device('JoystickDevice', 'rover')

JoystickDevice.storage.joystick1 = pygame.joystick.Joystick(0)
JoystickDevice.storage.joystick1.init()

@JoystickDevice.every(LOOP_PERIOD)
async def read_joystick():
    pygame.event.get()
    left = [JoystickDevice.storage.joystick1.get_axis(0), JoystickDevice.storage.joystick1.get_axis(1)]
    right = [JoystickDevice.storage.joystick1.get_axis(3), JoystickDevice.storage.joystick1.get_axis(4)]
    await JoystickDevice.publish('joystick1', left)
    await JoystickDevice.publish('joystick2', right)

try:
    JoystickDevice.start()
    JoystickDevice.wait()
except KeyboardInterrupt:
    JoystickDevice.stop()
