#! /bin/bash

# ssh into both pis, take pictures, and copy them to the cwd
PI1=10.0.0.5
PI2=10.0.0.6
USERNAME="pi"
FILENAME="panorama.jpg"
CMD="raspistill -o $FILENAME"

ssh pi@${PI1} $CMD
ssh pi@${PI2} $CMD

scp pi@${PI1}:$FILENAME "front_${FILENAME}"
scp pi@${PI2}:$FILENAME "back_${FILENAME}"

