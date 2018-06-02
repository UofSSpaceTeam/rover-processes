#Skylar Koroluk
#Some pieces borrowed from Irene 
#Obstacle Avoidance Plan B for USST rover 2018

import pyzed.camera as zcam
import pyzed.defines as sl
import pyzed.types as tp
import pyzed.core as core
import math
import time
import numpy as np
import sys
import cv2
from robocluster import Device
import config
log = config.getLogger()

AvoidanceDecision = Device('AvoidanceDecision', 'rover', network=config.network)

DISTANCE_THRESHOLD = 2 #meters
PIXEL_THRESHOLD = 30

#@AvoidanceDecision.task
def init():
    AvoidanceDecision.storage.zed = zcam.PyZEDCamera()
    print("testing")
    # Create a PyInitParameters object and set configuration parameters
    AvoidanceDecision.storage.init_params = zcam.PyInitParameters()
    AvoidanceDecision.storage.init_params.depth_mode = sl.PyDEPTH_MODE.PyDEPTH_MODE_QUALITY  # Use PERFORMANCE depth mode
    AvoidanceDecision.storage.init_params.coordinate_units = sl.PyUNIT.PyUNIT_MILLIMETER  # Use milliliter units (for depth measurements)

    # Open the camera
    err = AvoidanceDecision.storage.zed.open(AvoidanceDecision.storage.init_params)
    while err != tp.PyERROR_CODE.PySUCCESS:
        err = AvoidanceDecision.storage.zed.open(AvoidanceDecision.storage.init_params)

    # Create and set PyRuntimeParameters after opening the camera
    AvoidanceDecision.storage.runtime_parameters = zcam.PyRuntimeParameters()
    AvoidanceDecision.storage.runtime_parameters.sensing_mode = sl.PySENSING_MODE.PySENSING_MODE_FILL  #use fill mode
    # zed camera has to be open before importing pycuda
   
@AvoidanceDecision.task
async def main():

    """Takes in a sampled frame from the ZED camera and decides whether the rover needs to adjust its course
    returns either "right", "left" or None."""

    AvoidanceDecision.storage.zed = zcam.PyZEDCamera()
    # Create a PyInitParameters object and set configuration parameters
    AvoidanceDecision.storage.init_params = zcam.PyInitParameters()
    AvoidanceDecision.storage.init_params.depth_mode = sl.PyDEPTH_MODE.PyDEPTH_MODE_QUALITY  # Use PERFORMANCE depth mode
    AvoidanceDecision.storage.init_params.coordinate_units = sl.PyUNIT.PyUNIT_MILLIMETER  # Use milliliter units (for depth measurements)
    
    # Open the camera
    err = AvoidanceDecision.storage.zed.open(AvoidanceDecision.storage.init_params)
    
    failed = 0
    while err != tp.PyERROR_CODE.PySUCCESS:
        failed += 1
        print("\rCould not open camera", failed, "times")
        err = AvoidanceDecision.storage.zed.open(AvoidanceDecision.storage.init_params)

    # Create and set PyRuntimeParameters after opening the camera
    AvoidanceDecision.storage.runtime_parameters = zcam.PyRuntimeParameters()
    AvoidanceDecision.storage.runtime_parameters.sensing_mode = sl.PySENSING_MODE.PySENSING_MODE_FILL  #use fill mode
    
    # zed camera has to be open before importing pycuda
    #import pycuda.autoinit
    #import pycuda.driver as cuda

    #from pycuda.compiler import SourceModule
    while True:
        err = AvoidanceDecision.storage.zed.open(AvoidanceDecision.storage.init_params)
        start_time = time.time()
        runtime_parameters = AvoidanceDecision.storage.runtime_parameters 
        zed = AvoidanceDecision.storage.zed
        image = core.PyMat()
        depth = core.PyMat()
        point_cloud = core.PyMat()
        depth_display = core.PyMat()
        frame_capped = False
        count = 0
        while frame_capped==False:  # loops until it succesfully grabs a frame
            # A new image is available if grab() returns PySUCCES
            print("times falied grabbing an image: ", count)
            count += 1
            if zed.grab(runtime_parameters) == tp.PyERROR_CODE.PySUCCESS:
                start_logic_time = time.time()
                frame_capped = True
                # Retrieve left image
                zed.retrieve_image(image, sl.PyVIEW.PyVIEW_LEFT)
                #zed.retrieve_image(depth_display, sl.PyVIEW.PyVIEW_DEPTH)
                # Retrieve depth map. Depth is aligned on the left image
                #zed.retrieve_measure(depth, sl.PyMEASURE.PyMEASURE_DEPTH)
                # Retrieve colored point cloud. Point cloud is aligned on the left image.
                zed.retrieve_measure(point_cloud, sl.PyMEASURE.PyMEASURE_XYZRGBA)
                # We measure the distance camera - object using Euclidean distance
                # Break image into three zones vertically


                #mod = SourceModule(
                """
                #include <stdio.h>
                __global__ void filterPoints(int * result,float *points,int size, float minX,float maxX,float minY,float maxY,float maxZ)
                // Checks whether the point is within a certain cube.
                {
                const int i = threadIdx.x+blockIdx.x*blockDim.x;
                if (i<size)
                    result[i] = (points[3*i]>minX && points[3*i] <maxX && points[3*i+1]>minY && points[3*i+1] <maxY && points[3*i+2] <maxZ);
                }
                """
                #)

                width = image.get_width()
                height = image.get_height()
                log.debug("pixels: {}".format(width*height))
                zone_end = [round(width/3), round(width/3)*2, width]
                zone_start = [0, round(width/3), round(width/3)*2]
                zone_count = [0, 0, 0]
                for index in range(3):
                    for x in range(zone_start[index], zone_end[index], 10):
                        for y in range(0, round(height*2/3), 10):
                            err, point_cloud_value = point_cloud.get_value(x, y)
                            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                     point_cloud_value[1] * point_cloud_value[1] +
                                     point_cloud_value[2] * point_cloud_value[2])
                            distance = distance/1000 #convert it to meters manually (for now at least)
                            if np.isnan(distance):
                                distance = 0
                            if np.isinf(distance):
                                distance = 4
                            #print("Distance to Camera at ({0}, {1}): {2} mm\n".format(x, y, distance))
                            if distance > DISTANCE_THRESHOLD:
                                zone_count[index] += 1
                turn = None
                if zone_count[0] > zone_count[2]:
                    furthest = 0
                    furthest_value = zone_count[0]
                else:
                    furthest = 2
                    furthest_value = zone_count[2]
                if zone_count[1] > (furthest_value - PIXEL_THRESHOLD):
                    furthest = 1
                if furthest == 0:
                    turn = "left"
                elif furthest == 2:
                    turn = "right"


                sys.stdout.flush()


        # Close the camera
        #zed.close()
        total_time = time.time()-start_time
        logic_time = time.time()-start_logic_time
        startup_time = total_time-logic_time
        print("Total elapsed time: ", total_time)
        print("Startup time: ", startup_time)
        print("Logic time: ", logic_time)
        print("zone1: ", zone_count[0], "\nzone2: ", zone_count[1], "\nzone3: ", zone_count[2])
        print("turn", turn)
        #cv2.imshow("Frame", depth_display.get_data())
        #key = cv2.waitKey(1)
        await AvoidanceDecision.publish('DirectionToTurn', turn)
        await AvoidanceDecision.sleep(2)
#init()
AvoidanceDecision.start()
AvoidanceDecision.wait()
