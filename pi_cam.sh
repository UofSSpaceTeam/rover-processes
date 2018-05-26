FPS=30
H=1080
W=1440
B=2500000
EXP=auto
MM=average
HOST=10.0.0.4

if [ $# -lt 1 ]; then
    echo "Usage: pi_cam.sh {front|back} [host]"
    exit 1
fi

if [ $1 == "front" ]; then
    PORT=5300
    echo "Front camera on port $PORT" # These don't print?!...
elif [ $1 == "back" ]; then
    PORT=5301
    echo "Back camera on port $PORT"
fi

if [ $# -eq 2 ]; then
    HOST=$2
fi


raspivid -cs 0 --exposure $EXP -w $W -h $H -t 00 -fps $FPS -b $B -mm $MM -o - | gst-launch-1.0 fdsrc ! h264parse !  rtph264pay ! udpsink host=$HOST port=$PORT &
#raspivid -cs 1 --exposure $EXP -w $W -h $H -t 00 -fps $FPS -b $B -mm $MM -o - | gst-launch-1.0 fdsrc ! h264parse !  rtph264pay ! udpsink host=$HOST port=5310 &
