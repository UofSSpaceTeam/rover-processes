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
sdr.sample_rate = 2e6
sdr.freq_correction = -4
sdr.gain = "auto"
sdr.center_freq = 433.5e6
sdr.set_bandwidth(500e3)

p_max_avg = []
 
def print_data(data,obj):
    global p_max_avg
    s_avg = []
    s_pwr = 10 * np.log10(np.absolute(data))
    for x in range(1,len(data)-N,N):
        s_avg.append(sum(s_pwr[x:x+N-1]))
    p_max = max(s_avg)
    if (len(p_max_avg) < 3):
        p_max_avg.append(p_max)
    else:
        value = sum(p_max_avg)/float(len(p_max_avg))
        log.debug(value)
        @RDFDevice.task
        async def send_power():
            await RDFDevice.publish('YagiPower', value)
        p_max_avg = []

try:
    RDFDevice.start()
    sdr.read_samples_async(print_data,256)

except KeyboardInterrupt:
    sdr.cancel_read_async()
    sdr.close()
    print("cancelled")




