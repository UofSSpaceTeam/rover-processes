#Skylar Koroluk
#Obstacle Avoidance Plan B for USST rover 2018

import pyzed.camera as zcam
import pyzed.defines as sl
import pyzed.types as tp
import pyzed.core as core
import math
import numpy as np
import sys

DISTANCE_THRESHOLD = 6 #meters
PIXEL_THRESHOLD = 200 #if the number of pixels above the DISTANCE_THRESHOLD in zone1 (the middle of the rovers field of view) is greater than or equal to this value 

def avoidance_decision():
    """Takes in a sampled frame from the ZED camera and decides whether the rover needs to adjust its course 
    returns either "right", "left" or None."""
    # Create a PyZEDCamera object
    zed = zcam.PyZEDCamera()

    # Create a PyInitParameters object and set configuration parameters
    init_params = zcam.PyInitParameters()
    init_params.depth_mode = sl.PyDEPTH_MODE.PyDEPTH_MODE_PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.PyUNIT.PyUNIT_MILLIMETER  # Use milliliter units (for depth measurements)

    # Open the camera
    err = zed.open(init_params)
    if err != tp.PyERROR_CODE.PySUCCESS:
        exit(1)

    # Create and set PyRuntimeParameters after opening the camera
    runtime_parameters = zcam.PyRuntimeParameters()
    runtime_parameters.sensing_mode = sl.PySENSING_MODE.PySENSING_MODE_STANDARD  # Use STANDARD sensing mode

    # Capture 50 images and depth, then stop
    i = 0
    image = core.PyMat()
    depth = core.PyMat()
    point_cloud = core.PyMat()
    flag = True
    while flag:  # loops until it succesfully grabs a frame
        # A new image is available if grab() returns PySUCCESS
        if zed.grab(runtime_parameters) == tp.PyERROR_CODE.PySUCCESS:
            flag = False
            # Retrieve left image
            #zed.retrieve_image(image, sl.PyVIEW.PyVIEW_LEFT)
            # Retrieve depth map. Depth is aligned on the left image
            #zed.retrieve_measure(depth, sl.PyMEASURE.PyMEASURE_DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.PyMEASURE.PyMEASURE_XYZRGBA)

            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            # Break image into three zones vertically 
            width = image.get_width()
            height = image.get_height()
            zone_end = [round(width/3), round(width/3)*2, width]
            zone_start = [0, round(width/3), round(width/3)*2]
            zone_count = [0, 0, 0]
            for index in range(3):
                for x in range(zone_start[index], zone_end[index]):
                    for y in range(height):
                        err, point_cloud_value = point_cloud.get_value(x, y)
                        distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])
                        distance = distance/1000 #convert it to meters manually
                        if np.isnan(distance):
                            distance = 0
                        if np.isinf(distance):
                            distance = 40
                        distance = round(distance)
                        #print("Distance to Camera at ({0}, {1}): {2} mm\n".format(x, y, distance))
                        if distance > DISTANCE_THRESHOLD:
                            zone_count[index] += 1
            turn = None
            if zone_count[0] > zone_count[2]:
                furthest = 0
                furthest_value = zone_count[0]
            else:
                furthest = 2
                furthes_value = zone_count[2]
            if zone_count[1] > (furthest_value - PIXEL_THRESHOLD):
                furthest = 1
            if furthest == 0:
                turn = "left"
            elif furthest == 2:
                turn = "right"
        i = i + 1
    # Close the camera
    zed.close()
    return turn

def avoidance_override():
    """Takes in either "right" or "left. Turns "angle" degrees and drives "distance" forward."""
    

def obstacle_avoidance():
    avoid = avoidance_decision()
    if avoid is None:
        return
    else:
        avoidance_override(avoid)
        obstacle_avoidance()

    


