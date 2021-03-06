

import pyzed.camera as zcam
import pyzed.defines as sl
import pyzed.types as tp
import pyzed.core as core
import math
import numpy as np
import sys
import cv2
import time

DISTANCE_THRESHOLD = 6 #meters
PIXEL_THRESHOLD = 200 

def main():
   
    # Create a PyZEDCamera object
    zed = zcam.PyZEDCamera()
    
    # Create a PyInitParameters object and set configuration parameters
    init_params = zcam.PyInitParameters()
    init_params.depth_mode = sl.PyDEPTH_MODE.PyDEPTH_MODE_QUALITY  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.PyUNIT.PyUNIT_MILLIMETER  # Use milliliter units (for depth measurements)
    
    # Open the camera
    err = zed.open(init_params)
    if err != tp.PyERROR_CODE.PySUCCESS:
        print('Could not open camera')
        exit(1) 
    
    
    # Create and set PyRuntimeParameters after opening the camera
    runtime_parameters = zcam.PyRuntimeParameters()
    runtime_parameters.sensing_mode = sl.PySENSING_MODE.PySENSING_MODE_FILL  # Use STANDARD sensing mode

    image = core.PyMat()
    depth = core.PyMat()
    point_cloud = core.PyMat()
    depth_display = core.PyMat()
    frame_capped = False    
    while frame_capped==False:
        starttime = time.time()
        # A new image is available if grab() returns PySUCCESS
        if zed.grab(runtime_parameters) == tp.PyERROR_CODE.PySUCCESS:
            frame_capped = True
            # Retrieve left image
            zed.retrieve_image(image, sl.PyVIEW.PyVIEW_LEFT)
	    #retrieve depth image
            zed.retrieve_image(depth_display, sl.PyVIEW.PyVIEW_DEPTH)
            # Retrieve depth map. Depth is aligned on the left image
            zed.retrieve_measure(depth, sl.PyMEASURE.PyMEASURE_DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.PyMEASURE.PyMEASURE_XYZRGBA)          
            cv2.imshow("depth map", depth_display.get_data()) 
            key = cv2.waitKey(1)
            # We measure the distance camera - object using Euclidean distance
            # Break image into three zones vertically 
            width = image.get_width()
            height = image.get_height()
            print("pixels", width*height)
            zone_end = [round(width/3), round(width/3)*2, width]
            zone_start = [0, round(width/3), round(width/3)*2]
            zone_count = [0, 0, 0]
            i = 0
            for index in range(3):
                for x in range(zone_start[index], zone_end[index], 5):
                    for y in range(0, height, 5):
                        i += 1
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
 
              
            print(time.time()-starttime)
            
            sys.stdout.flush()

    # Close the camera
    zed.close()
    print("Testing")
    print("zone1: ", zone_count[0], "\nzone2: ", zone_count[1], "\nzone3: ", zone_count[2])
    print("turn", turn)
    return turn



if __name__ == "__main__":
    main()
