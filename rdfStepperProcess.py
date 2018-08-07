import RPi.GPIO as GPIO
import time 
from robocluster import Device
import config


## Define RPi GPIO Pins
out1 = 13
out2 = 11
out3 = 15
out4 = 12

## Define Positional Constants
i=0
positive=0
negative=0
y=0

## Set GPIO Pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(out1,GPIO.OUT)
GPIO.setup(out2,GPIO.OUT)
GPIO.setup(out3,GPIO.OUT)
GPIO.setup(out4,GPIO.OUT)

# print("First calibrate by giving some +ve and -ve values.....")

## Create Robocluster Device
StepperDevice = Device('RDFStepperMotor', 'rover', network=config.network)

@StepperDevice.on('*/StepperRotation')
async def rotate_stepper(StepperRotation, data):
    """
    Handles rotating the RDF Yagi Antenna based on WebUI Input
    """
    x = int(data)
    GPIO.output(out1,GPIO.LOW)
    GPIO.output(out2,GPIO.LOW)
    GPIO.output(out3,GPIO.LOW)
    GPIO.output(out4,GPIO.LOW)
    x = input()
    if x>0 and x<=400:
        for y in range(x,0,-1):
            if negative==1:
                if i==7:
                    i=0
                else:
                    i=i+1
                y=y+2
                negative=0
            positive=1
            if i==0:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==1:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==2:  
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==3:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==4:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==5:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.HIGH)
                time.sleep(0.03)
            elif i==6:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.HIGH)
                time.sleep(0.03)
            elif i==7:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.HIGH)
                time.sleep(0.03)
            if i==7:
                i=0
                continue
            i=i+1
        
    elif x<0 and x>=-400:
        x=x*-1
        for y in range(x,0,-1):
            if positive==1:
                if i==0:
                    i=7
                else:
                    i=i-1
                y=y+3
                positive=0
            negative=1
            if i==0:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==1:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==2:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==3:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.HIGH)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==4:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.LOW)
                time.sleep(0.03)
            elif i==5:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.HIGH)
                GPIO.output(out4,GPIO.HIGH)
                time.sleep(0.03)
            elif i==6:
                GPIO.output(out1,GPIO.LOW)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.HIGH)
                time.sleep(0.03)
            elif i==7:
                GPIO.output(out1,GPIO.HIGH)
                GPIO.output(out2,GPIO.LOW)
                GPIO.output(out3,GPIO.LOW)
                GPIO.output(out4,GPIO.HIGH)
                time.sleep(0.03)
            if i==0:
                i=7
                continue
            i=i-1 

StepperDevice.start()
StepperDevice.wait()
