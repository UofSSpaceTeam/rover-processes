

import pyzed.camera as zcam
import pyzed.defines as sl
import pyzed.types as tp
import pyzed.core as core
import math
import numpy as np
import sys
import cv2
import time


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
        print('error')
        exit(1) 
    # Zed camera has to be open first before importing pycuda
    import pycuda.autoinit
    import pycuda.driver as cuda

    from pycuda.compiler import SourceModule
    
    
    # Create and set PyRuntimeParameters after opening the camera
    runtime_parameters = zcam.PyRuntimeParameters()
    runtime_parameters.sensing_mode = sl.PySENSING_MODE.PySENSING_MODE_FILL  # Use STANDARD sensing mode

    image = core.PyMat()
    depth = core.PyMat()
    point_cloud = core.PyMat()
    depth_display = core.PyMat()
    
    while True:
        starttime = time.time()
        # A new image is available if grab() returns PySUCCESS
        if zed.grab(runtime_parameters) == tp.PyERROR_CODE.PySUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.PyVIEW.PyVIEW_LEFT)
	    #retrieve depth image
            zed.retrieve_image(depth_display, sl.PyVIEW.PyVIEW_DEPTH)
            # Retrieve depth map. Depth is aligned on the left image
            zed.retrieve_measure(depth, sl.PyMEASURE.PyMEASURE_DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.PyMEASURE.PyMEASURE_XYZRGBA)
            
            mod = SourceModule("""
            #include <stdio.h>
            __global__ void filterPoints(int * result,float *points,int size, float minX,float maxX,float minY,float maxY,float maxZ)
            // Checks whether the point is within a certain cube.
            {
            const int i = threadIdx.x+blockIdx.x*blockDim.x;
            if (i<size)
                result[i] = (points[3*i]>minX && points[3*i] <maxX && points[3*i+1]>minY && points[3*i+1] <maxY && points[3*i+2] <maxZ);
            }
            """)

            

            points = point_cloud.get_data()
            points = points.reshape(-1,4)
            points = points[:,0:3] # get rid of the alpha field
            points = points[~np.isnan(points).any(axis=1)] # Filter out non values
            points = points.astype(np.float32)
            points_gpu = cuda.mem_alloc(points.nbytes)
            cuda.memcpy_htod(points_gpu,points) # copy points into GPU
            dataSize = points[:,0].shape[0]
            result = np.zeros(dataSize,dtype = np.int32)
            result_gpu = cuda.mem_alloc(result.nbytes) # allocate memory for the result
            filterFunc = mod.get_function('filterPoints')
            gridX = int(dataSize/1024)+1
            xmin = -1000
            xmax = 1000
            ymin = -1000
            ymax = 1000
            zmax = 5000
            filterFunc(
                    result_gpu,
                    points_gpu,np.int32(dataSize),
                    np.float32(xmin),
                    np.float32(xmax),
                    np.float32(ymin),
                    np.float32(ymax),
                    np.float32(zmax),
                    block=(1024,1,1),
                    grid=(gridX, 1,1),
                    shared = 0)
            cuda.memcpy_dtoh(result,result_gpu) # copy result from GPU
            # Each point in result is Bool: 0: not in the cube, 1: in the cube.
            print(sum(result))
            

            #get the distance in mm
            #distances = np.sqrt(points[:,:,0]*points[:,:,0]+points[:,:,1]*points[:,:,1]+points[:,:,2]*points[:,:,2])
            #find the closest point
            #min_index = np.unravel_index(np.nanargmin(distances),distances.shape)
            #print("closest object at ({0},{1},{2})mm".format(points[min_index[0],min_index[1],0],points[min_index[0],min_index[1],1],points[min_index[0],min_index[1],2]))
            cv2.imshow("depth map", depth_display.get_data()) 
            key = cv2.waitKey(5)
              
            print(time.time()-starttime)
            
            sys.stdout.flush()

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()
