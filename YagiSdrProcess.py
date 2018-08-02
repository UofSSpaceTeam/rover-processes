from rtlsdr import RtlSdr
import matplotlib.pyplot as plt
import numpy as np

sdr = RtlSdr(serial_number='00000001')


sdr.sample_rate = 2.048e6
#sdr.center_freq = 462.5625e6
sdr.freq_correction = -4
sdr.gain = "auto"
sdr.center_freq = 434e6

sdr.set_bandwidth(8000)

samples = sdr.read_samples(256*1024)
sdr.close()

power,psdFreq = plt.psd(samples,NFFT=1024,Fs=sdr.sample_rate/1e6,Fc=sdr.center_freq/1e6)
plt.xlabel("freq")
plt.ylabel("Power")

plt.show()
#print("getting serial")
print(samples)
#print(RtlSdr.get_device_serial_addresses())

powerDb = 10*np.log10(power)

maxPower = max(powerDb)
maxInd =powerDb.tolist().index(maxPower)
maxFreq = psdFreq[maxInd]

print(maxPower)
print(maxFreq)


