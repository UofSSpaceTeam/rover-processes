gst-launch-1.0 -v udpsrc port=5301\
    ! application/x-rtp, payload=96\
    ! rtph264depay\
    ! avdec_h264\
    ! autovideosink sync=false
