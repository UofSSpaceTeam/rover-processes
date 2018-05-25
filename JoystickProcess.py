import time
import os
from roverutil import getnetwork

import pygame
from robocluster import Device

LOOP_PERIOD = 0.1 # seconds
#for is button pressed
ButtonDown = False
ButtonNum = 4

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
        trigger = 2
    else:
        rightHoriz = 3
        rightVert = 4
        trigger = [2,5]
    return rightHoriz, rightVert, trigger

ButtonDown = False
ButtonNum = 4

def IsButtonPressed(ButtonNum, ButtonDown):
    ButtonBool = JoystickDevice.storage.joystick1.get_button(ButtonNum)
    #if program doesnt know button is pressed and it is, then change ButtonDown to True
    if ButtonDown == False:
        if ButtonBool == True:
            print("button " + str(ButtonNum) + " is down")
            ButtonDown = True
        else:
            pass
    else:
        pass

    if ButtonDown == True:
        #if program registers button as down but it isnt anymore, change ButtonDown to False
        if ButtonBool == False:
            print("button " + str(ButtonNum) + " is up")
            ButtonDown = False

        else:
            #otherwise do nothing so the action the button triggers is only done once
            pass
    else:
        pass
    # so function knows previous status of button next loop
    return ButtonDown

done = False

while done == False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get():  # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            done = True  # Flag that we are done so we exit this loop
    ButtonDown = IsButtonPressed(ButtonNum, ButtonDown)

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
