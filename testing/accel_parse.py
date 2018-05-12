import sys
import re
import json
from statistics import variance, mean, stdev


f_name = sys.argv[1]

measurements = []

with open(f_name, 'r') as f:
    for line in f.readlines():
        if re.search('accelerometer', line):
            entry_s = re.sub('BNO055/BNO055/accelerometer ', '', line)
            entry_s = re.sub('\'', '"', entry_s)
            measurements.append(json.loads(entry_s))

z_vals = [entry['z'] for entry in measurements]
y_vals = [entry['y'] for entry in measurements]
x_vals = [entry['x'] for entry in measurements]
print("Mean: x={}, y={}, z={}".format(mean(x_vals), mean(y_vals), mean(z_vals)))
print("Standard Dev: x={}, y={}, z={}".format(
    stdev(x_vals), stdev(y_vals), stdev(z_vals)
    ))


