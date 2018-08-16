from rtlsdr import RtlSdr
import time
import numpy as np
import random

from robocluster import Device
import config
log = config.getLogger()

RDFDevice = Device('RDF', 'rover', network=config.network)

sdr = RtlSdr(serial_number='00000001')

N = 8
# threshold for edge trigger
threshold = 10
sdr.sample_rate = 2e6
sdr.freq_correction = -4
sdr.gain = "auto"
sdr.center_freq = 433.5e6
sdr.set_bandwidth(500e3)

p_max_avg = []
prev_value = 0
rising_time = 0
falling_time = 0

def print_data(data,obj):
    global p_max_avg
    global prev_value
    global rising_time

    s_avg = []
    s_pwr = 10 * np.log10(np.absolute(data))
    for x in range(1,len(data)-N,N):
        s_avg.append(sum(s_pwr[x:x+N-1]))
    p_max = max(s_avg)
    if (len(p_max_avg) < 3):
        p_max_avg.append(p_max)
    else:
        value = sum(p_max_avg)/float(len(p_max_avg))

        if value - prev_value - threshold > 0:
            # rising
            rising_time = time.time()
        elif value - prev_value + threshold < 0:
            # falling
            td = (time.time() - rising_time)/1000

            # send time diff back to server here
            @RDFDevice.task
            async def send_difference():
                await RDFDevice.publish('TimeDifference', td)

        log.debug(value)
        @RDFDevice.task
        async def send_power():
            await RDFDevice.publish('YagiPower', value)
        p_max_avg = []
        prev_value = value

try:
    RDFDevice.start()
    sdr.read_samples_async(print_data,256)

except KeyboardInterrupt:
    sdr.cancel_read_async()
    sdr.close()
    print("cancelled")
