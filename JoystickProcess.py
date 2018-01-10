import time
import os
import pygame
from threading import Thread, BoundedSemaphore
from robocluster import device

LOOP_PERIOD = 100

os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
pygame.joystick.init()

JoystickDevice = Device('JoystickDevice', 'demo_device')

JoystickDevice.storage.joystick1 = pygame.joystick.Joystick(0)
JoystickDevice.storage.joystick1.init()

@JoystickDevice.every(LOOP_PERIOD)
async def every():
	pygame.event.get()
	left = [JoystickDevice.storage.joystick1.get_axis(0), JoystickDevice.joystick1.get_axis(1)]
	right = [JoystickDevice.storage.joystick1.get_axis(3), JoystickDevice.joystick1.get_axis(4)]
	await JoystickDevice.publish('joystick1', left)
	await JoystickDevice.publish('joystick2', right)

JoystickDevice.run()
