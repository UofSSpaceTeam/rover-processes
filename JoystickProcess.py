import time
import os

import pygame
from robocluster import Device

import config
log = config.getLogger()

LOOP_PERIOD = 0.1 # seconds

os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
pygame.joystick.init()

JoystickDevice = Device('JoystickDevice', 'rover', network=config.network)

num_controllers = pygame.joystick.get_count()
log.debug('Number of connected controllers: {}'.format(num_controllers))
JoystickDevice.storage.controllers = []
for num in range(0, num_controllers):
    joystick = pygame.joystick.Joystick(num)
    joystick.init()
    JoystickDevice.storage.controllers.append(joystick)

button_names = {'buttonA':0, 'buttonB':1, 'buttonX':2, 'buttonY':3,
        'bumperL':4, 'bumperR':5}
JoystickDevice.storage.last_state = [{key:0 for key in button_names.keys()}]*num_controllers

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
    for i, controller in enumerate(JoystickDevice.storage.controllers):
        rightHor = buttonAssign()[0]
        rightVer = buttonAssign()[1]
        trigger = buttonAssign()[2]
        pygame.event.get()
        left = [controller.get_axis(0), -controller.get_axis(1)]
        right = [controller.get_axis(rightHor), -controller.get_axis(rightVer)]
        # windows treats two triggers as one value, so this code makes linux behave in the same way for the sake of consistency
        if os.name == 'nt':
            trig = controller.get_axis(trigger)
        else:
            ltrig = (1 + controller.get_axis(trigger[0]))/2
            rtrig = (1 + controller.get_axis(trigger[1]))/2
            trig = rtrig - ltrig


        await JoystickDevice.publish('controller{}/joystick1'.format(i), left)
        await JoystickDevice.publish('controller{}/joystick2'.format(i), right)
        await JoystickDevice.publish('controller{}/trigger'.format(i), trig)

        for button in button_names.keys():
            button_val = controller.get_button(button_names[button])
            last_value = JoystickDevice.storage.last_state[i][button]
            if button_val == 1 and last_value == 0:
                await JoystickDevice.publish('controller{}/{}_down'.format(i, button), button_val)
            elif button_val == 0 and last_value == 1:
                await JoystickDevice.publish('controller{}/{}_up'.format(i, button), button_val)
            await JoystickDevice.publish('controller{}/{}'.format(i, button), button_val)
            JoystickDevice.storage.last_state[i][button] = button_val

        dpad = controller.get_hat(0)
        await JoystickDevice.publish('controller{}/dpad'.format(i), dpad)

        log.debug('left stick: ' + str(left))
        log.debug('right stick: ' + str(right))
        log.debug('trigger: ' + str(trig))

try:
    JoystickDevice.start()
    JoystickDevice.wait()
except KeyboardInterrupt:
    JoystickDevice.stop()
