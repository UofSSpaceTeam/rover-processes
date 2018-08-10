from rtlsdr import RtlSdr

#import matplotlib.pyplot as plt
import asyncio


import numpy as np

async def streaming():

    sdr = RtlSdr(serial_number='00000001')

    sdr.sample_rate = 2.048e6
    #sdr.center_freq = 462.5625e6
    sdr.freq_correction = -4
    sdr.gain = "auto"
    sdr.center_freq = 433.5e6

    sdr.set_bandwidth(500e3)

    async for samples in sdr.stream():
        print(samples)

    await sdr.stop()

    sdr.close()

loop = asyncio.get_event_loop()
loop.run_until_complete(streaming())

















def print_data(data,obj):
    #print(data)
    for x in range(len(data)):
        I = np.real(data[x])
        Q = np.imag(data[x])
        P_dBm = 10 * np.log10(10 * (np.power(I,2) + np.power(Q,2)))
        if(x%100 == 0):
            print(P_dBm)


