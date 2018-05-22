import utm
import statistics as stats
import numpy as np

northing = []
easting = []

string_path = "/Users/skylar/Desktop/raw_rover_data_lat"
f = open(string_path)  # opens text file
file_text = f.read()  # creates a string with contents of text file

latitudes = file_text.split()

string_path = "/Users/skylar/Desktop/raw_rover_data_long"
f = open(string_path)  # opens text file
file_text = f.read()  # creates a string with contents of text file

longitudes = file_text.split()

for index in range(len(latitudes)):
    latitude_value = float(latitudes[index])
    longitude_value = float(longitudes[index])
    northing_value = utm.from_latlon(latitude_value, longitude_value)[0]
    easting_value = utm.from_latlon(latitude_value, longitude_value)[1]
    northing.append(northing_value)
    easting.append(easting_value)
def find_mean(list):
    N = len(list)
    sum = 0
    for index in range(N):
        sum += list[index]
    mean = sum/N
    return mean

def get_rid_of_mean(list, mean):
    for index in range(len(list)):
        list[index] = list[index] - mean
    return list

average_northing = find_mean(northing)
average_easting = find_mean(easting)
northing_noise = get_rid_of_mean(northing, average_northing)
easting_noise = get_rid_of_mean(easting, average_easting)

print(average_northing)
print(average_easting)

print(northing_noise)
print(easting_noise)

variance_of_northing = stats.variance(northing, average_northing)
variance_of_easting = stats.variance(easting, average_easting)
R = np.cov(easting, northing)
print(variance_of_northing)
print(variance_of_easting)
print(R)