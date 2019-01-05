#! /usr/bin/python
from evdev import InputDevice, categorize, ecodes
import RPi.GPIO as GPIO
#setup of pins and config
deadzone = 5

chan_list = [33,35,36,38]
frequency = 60
GPIO.setmode(GPIO.BOARD)
GPIO.setup(chan_list, GPIO.OUT)
GPIO.setup([37,40], GPIO.OUT)

GPIO.output(chan_list, GPIO.LOW)

pwm1 = GPIO.PWM(37,frequency)
pwm2 = GPIO.PWM(40,frequency)
pwm1.start(0)
pwm2.start(0)

gamepad = InputDevice('/dev/input/event0')
print(gamepad)
x = 0
while x == 0:
	for event in gamepad.read_loop():
	
	
		value = float(event.value)/33000
		if event.code == 1:
			left_value = value
			left_direction = 0
			left_absvalue = abs(value)*100
			pwm1.ChangeDutyCycle(left_absvalue)
			if left_value > 0 and left_absvalue > deadzone:
				left_direction = 1
			elif left_value < 0 and left_absvalue > deadzone:
				left_direction = -1
		
			print('Left','Direction',left_direction,'Val',left_absvalue)
			if left_direction == -1:
				GPIO.output(36,GPIO.HIGH)
				GPIO.output(38, GPIO.LOW)
				print('Forward')
			if left_direction == 1:
				GPIO.output(38,GPIO.HIGH)
				GPIO.output(36, GPIO.LOW)
				print('Backward')
		if event.code == 4:
			right_value = value
			right_direction = 0
			right_absvalue = abs(value)*100
			pwm2.ChangeDutyCycle(right_absvalue)
			if right_value > 0 and right_absvalue > deadzone:
				right_direction = 1
			elif right_value < 0 and right_absvalue > deadzone:
				right_direction = -1
		
			print('Right','Direction',right_direction,'Val',right_absvalue)
			if right_direction == -1:
				GPIO.output(35,GPIO.HIGH)
				GPIO.output(33, GPIO.LOW)
				print('Forward')
			if right_direction == 1:
				GPIO.output(33,GPIO.HIGH)
				GPIO.output(35, GPIO.LOW)
				print('Backward')
		if event.code == 305:
			print('B')
			x = 1
			print(x)
			break
			
		
GPIO.cleanup()
#sudo /usr/bin/python  /home/pi/MiniRoverController.py  &

