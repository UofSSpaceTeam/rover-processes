import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BOARD)
GPIO.setup(03, GPIO.OUT)

pwm=GPIO.PWM(03, 20) #pin 3 @ 20 hertz (the 20hz basically controls voltage and speed)
pwm.start(0)

try:
	while True:
		pwm.ChangeDutyCycle(2.5)
		sleep(2)
		pwm.ChangeDutyCycle(7.5)
		sleep(2)
		pwm.ChangeDutyCycle(12.5)
		sleep(2)
except KeyboardInterrupt:
	pwm.stop()
	GPIO.cleanup()

'''def SetAngle(angle):
	duty = angle / 18 + 2
	GPIO.output(03, True)
	pwm.ChangeDutyCycle(duty)
	sleep(1)
	GPIO.output(03, False)
	pwm.ChangeDutyCycle(0)
SetAngle(90'
pwm.stop()
GPIO.cleanup()'''
