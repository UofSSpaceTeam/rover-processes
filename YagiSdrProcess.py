from rtlsdr import RtlSdr
#import matplotlib.pyplot as plt

import numpy as np
sdr = RtlSdr(serial_number='00000001')
#sdr = RtlSdr()

sdr.sample_rate = 2.048e6
#sdr.center_freq = 462.5625e6
sdr.freq_correction = -4
sdr.gain = "auto"
sdr.center_freq = 433.5e6

sdr.set_bandwidth(500e3)

#samples = sdr.read_samples(256*1024)
#sdr.close()

#power,psdFreq = plt.psd(samples,NFFT=1024,Fs=sdr.sample_rate/1e6,Fc=sdr.center_freq/1e6)
#plt.xlabel("freq")
#plt.ylabel("Power")

#plt.show()
#print("getting serial")
#print(samples)
#print(RtlSdr.get_device_serial_addresses())

#powerDb = 10*np.log10(power)

#maxPower = max(powerDb)
#maxInd =powerDb.tolist().index(maxPower)
#maxFreq = psdFreq[maxInd]

#print(maxPower)
#print(maxFreq)


#fft = np.fft.fft(samples)

#freqs = np.fft.fftfreq(len(samples))


def print_data(data,obj):
    #print(data)
    for x in range(len(data)):
        I = np.real(data[x])
        Q = np.imag(data[x])
        P_dBm = 10 * np.log10(10 * (np.power(I,2) + np.power(Q,2)))
        print(P_dBm)

sdr.read_samples_async(print_data,256)



