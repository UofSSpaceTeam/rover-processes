from multiprocessing import Process,Queue
import pyzed.camera as zcam
import pyzed.defines as sl
import pyzed.types as tp
import pyzed.core as core
import math
import numpy as np
import sys
import cv2

WIDTH = 640
HEIGHT = 360
def retriveDepth(q):
   
    # Create a PyZEDCamera object
    zed = zcam.PyZEDCamera()
    
    # Create a PyInitParameters object and set configuration parameters
    init_params = zcam.PyInitParameters()
    init_params.depth_mode = sl.PyDEPTH_MODE.PyDEPTH_MODE_PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.PyUNIT.PyUNIT_MILLIMETER  # Use milliliter units (for depth measurements)
    
    # Open the camera
    err = zed.open(init_params)
    if err != tp.PyERROR_CODE.PySUCCESS:
        print('error')
        exit(1)

    # Create and set PyRuntimeParameters after opening the camera
    runtime_parameters = zcam.PyRuntimeParameters()
    runtime_parameters.sensing_mode = sl.PySENSING_MODE.PySENSING_MODE_STANDARD  # Use STANDARD sensing mode

    # Capture 50 images and depth, then stop
    i = 0
    image = core.PyMat()
    depth = core.PyMat()
    point_cloud = core.PyMat()
    depth_display = core.PyMat()
    
    while True:
        # A new image is available if grab() returns PySUCCESS
        if zed.grab(runtime_parameters) == tp.PyERROR_CODE.PySUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.PyVIEW.PyVIEW_LEFT)
	    #retrieve depth image
            zed.retrieve_image(depth_display, sl.PyVIEW.PyVIEW_DEPTH)
            # Retrieve depth map. Depth is aligned on the left image
            #zed.retrieve_measure(depth, sl.PyMEASURE.PyMEASURE_DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.PyMEASURE.PyMEASURE_XYZRGBA,width=WIDTH,height=HEIGHT)
            
            points = point_cloud.get_data()
            
            #points = points.reshape(-1,4)
            #points = points[:,0:3]
            #points = points[~np.isnan(points).any(axis=1)]
            if q.empty():
                q.put(points)
            #points = points.astype(np.float32)
            cv2.imshow("depth map", depth_display.get_data()) 
            #key = cv2.waitKey(5)
            #print(i)
            #i = i + 1
            
            #sys.stdout.flush()

    # Close the camera
    zed.close()

def filterDepth(q):
    #import time
    #time.sleep(10)
    import pycuda.driver as cuda
    import pycuda.autoinit
    
    from pycuda.compiler import SourceModule
    mod = SourceModule("""
    #include <math.h>
    __global__ void findMin(float *result,float *points,int blockX, int blockY,int sizeY)
    {
      int idx = threadIdx.x+blockIdx.x*blockDim.x;
      int idy = threadIdx.y+blockIdx.y*blockDim.y;
      float minZ = 20;
      float minX = 20;
      float minY = 20;
      int i,j;
      for(i = idx * blockX; i < idx * blockX + blockX; i++)
          for(j = idy * blockY; j < idy * blockY + blockY; j++){
              if(points[2+3*(j+sizeY*i)] > 0 && points[2+3*(j+sizeY*i)] < minZ){
                  minZ = points[2+3*(j+sizeY*i)];
                  minX = points[0+3*(j+sizeY*i)];
                  minY = points[1+3*(j+sizeY*i)];
        }     
        }
     result[0+5*(idy+sizeY*idx)] = minX;
     result[1+5*(idy+sizeY*idx)] = minX;
     result[2+5*(idy+sizeY*idx)] = minY;
     result[3+5*(idy+sizeY*idx)] = minY;
     result[4+5*(idy+sizeY*idx)] = minZ;
    }
    __global__ void merge(float *result, int *flag, float *blocks,int sizeY,float threshold){
      int idx = threadIdx.x+blockIdx.x*blockDim.x;
      int idy = threadIdx.y+blockIdx.y*blockDim.y;
      int i = 2*idx;
      int j = 2*idy;
      if(abs(3*result[4+5*(idy+sizeY*idx)]-result[4+5*(idy+1+sizeY*idx)]-result[4+5*(idy+sizeY*(idx+1))]result[4+5*(idy+1+sizeY*(idx+1))]) < threshold){
        result[0+5*(idy+sizeY*idx)] = min(result[0+5*(idy+sizeY*idx)],result[0+5*(idy+sizeY*idx)],result[0+5*(idy+sizeY*idx)],result[0+5*(idy+sizeY*idx)]);
        result[1+5*(idy+sizeY*idx)] = max(result[1+5*(idy+sizeY*idx)],result[1+5*(idy+sizeY*idx)],result[1+5*(idy+sizeY*idx)],result[1+5*(idy+sizeY*idx)]);
        result[2+5*(idy+sizeY*idx)] = min(result[2+5*(idy+sizeY*idx)],result[2+5*(idy+sizeY*idx)],result[2+5*(idy+sizeY*idx)],result[2+5*(idy+sizeY*idx)]);
        result[3+5*(idy+sizeY*idx)] = max(result[3+5*(idy+sizeY*idx)],result[3+5*(idy+sizeY*idx)],result[3+5*(idy+sizeY*idx)],result[3+5*(idy+sizeY*idx)]);
        result[4+5*(idy+sizeY*idx)] = min(result[4+5*(idy+sizeY*idx)],result[4+5*(idy+sizeY*idx)],result[4+5*(idy+sizeY*idx)],result[4+5*(idy+sizeY*idx)]);
        flag[idy+sizeY*idx] = 1;
        flag[idy+1+sizeY*idx] = 1;
        flag[idy+sizeY*(idx+1)] = 1;
        flag[idy+1+sizeY*(idx+1)] = 1;
      }
    }    
    
    """)
    while True:
        if not q.empty():
            points = q.get()  
            points = points[120:240,20:620,0:3]/1000
            points = points.astype(np.float32)
            points_gpu = cuda.mem_alloc(points.nbytes)
            cuda.memcpy_htod(points_gpu,points)
            blocks = np.zeros((8,24,5),dtype = np.float32)#minX,maxX,minY,maxY,minZ
            block_gpu = cuda.mem_alloc(blocks.nbytes)
            findMinFunc = mod.get_function('findMin')
            findMinFunc(block_gpu,points_gpu,np.int32(15),np.int32(25),np.int32(600),block = (8,24,1),grid = (1,1,1),shared = 0)
            for i in range(3):
                
                
            

    
    
if __name__ == '__main__':
    q=Queue()
    
    imageProcess=Process(target=retriveDepth,args=(q,))
    filterProcess = Process(target=filterDepth,args=(q,))
    imageProcess.start()
    filterProcess.start()
    imageProcess.join()
    filterProcess.join()