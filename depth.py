from multiprocessing import Process,Queue
import pyzed.camera as zcam
import pyzed.defines as sl
import pyzed.types as tp
import pyzed.core as core
import math
import numpy as np
import sys
import cv2
import time

WIDTH = 1280
HEIGHT = 720
THRESHOLD = 0.1
def retriveDepth(q):
   
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

    # Create and set PyRuntimeParameters after opening the camera
    runtime_parameters = zcam.PyRuntimeParameters()
    runtime_parameters.sensing_mode = sl.PySENSING_MODE.PySENSING_MODE_FILL  # Use STANDARD sensing mode

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
            #zed.retrieve_image(depth_display, sl.PyVIEW.PyVIEW_DEPTH)
            
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
            #cv2.imshow("depth map", depth_display.get_data()) 
            #key = cv2.waitKey(5)
            #print(i)
            #i = i + 1
            
            #sys.stdout.flush()

    # Close the camera
    zed.close()

def filterDepth(q):
    
    import pycuda.driver as cuda
    import pycuda.autoinit
    
    from pycuda.compiler import SourceModule
    
    mod = SourceModule("""
    #include <math.h>
    #include <stdio.h>
    __global__ void findMinInBlock(float *result,float *points,int blockX, int blockY,int sizeY)
    {
      int idx = threadIdx.x+blockIdx.x*blockDim.x;
      int idy = threadIdx.y+blockIdx.y*blockDim.y;
      float minZ = 20;
      float minX = 20;
      float minY = 20;
      int i,j;
      for(i = idx * blockX; i < idx * blockX + blockX; i++)
          for(j = idy * blockY; j < idy * blockY + blockY; j++){
              if(points[2+3*(j+sizeY*i)] > 0.7 && points[2+3*(j+sizeY*i)] < minZ){
                  
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
    
    """)
    
    points_gpu = cuda.mem_alloc(np.zeros((240,1200,3),dtype = np.float32).nbytes)
    block_gpu = cuda.mem_alloc(np.zeros((16,48,5),dtype = np.float32).nbytes)
    while True:
        if not q.empty():
            starttime = time.time()
            points = q.get()  
            points = points[240:480,40:1240,0:3]/1000
            points = points.astype(np.float32)
            points[np.isnan(points)] = 20
            points[np.isinf(points)] = 20
            
            cuda.memcpy_htod(points_gpu,points)
            blocks = np.ones((16,48,5),dtype = np.float32)#minX,maxX,minY,maxY,minZ
            cuda.memcpy_htod(block_gpu,blocks)
            findMinFunc = mod.get_function('findMinInBlock')
            findMinFunc(block_gpu,points_gpu,np.int32(15),np.int32(25),np.int32(48),block = (16,48,1),grid = (1,1,1),shared = 0)
            cuda.memcpy_dtoh(blocks,block_gpu)
            blockNumX = 16
            blockNumY = 48
            flags_out = [[True for i in range(blockNumY)] for j in range(blockNumX)]
            objlist = []
            for ite in range(4):
                    blockNumX = int(blockNumX/2)
                    blockNumY = int(blockNumY/2)
                    points = blocks
                    blocks = np.zeros((blockNumX,blockNumY,5),dtype = np.float32)
                    flags_in = flags_out
                    flags_out = [[False for i in range(blockNumY)] for j in range(blockNumX)]
                    #print(points)
                    for i in range(blockNumX):
                        for j in range(blockNumY):
                            if flags_in[2*i][2*j] and flags_in[2*i+1][2*j] and flags_in[2*i][2*j+1] and flags_in[2*i+1][2*j+1]:
                                varZ = np.var(points[(2*i):(2*i+2),(2*j):(2*j+2),4])
                                #print(points[(2*i):(2*i+2),(2*j):(2*j+2),4])
                                #print(varZ)
                                if varZ < THRESHOLD:
                                    blocks[i,j,0] = np.min(points[(2*i):(2*i+2),(2*j):(2*j+2),0])
                                    blocks[i,j,1] = np.max(points[(2*i):(2*i+2),(2*j):(2*j+2),1])
                                    blocks[i,j,2] = np.min(points[(2*i):(2*i+2),(2*j):(2*j+2),2])
                                    blocks[i,j,3] = np.max(points[(2*i):(2*i+2),(2*j):(2*j+2),3])
                                    blocks[i,j,4] = np.min(points[(2*i):(2*i+2),(2*j):(2*j+2),4])
                                    flags_out[i][j] = True
                                else:
                                    #print(ite)
                                    objlist.append((points[2*i,2*j,0],points[2*i,2*j,1],points[2*i,2*j,2],points[2*i,2*j,3],points[2*i,2*j,4]))
                                    objlist.append((points[2*i+1,2*j,0],points[2*i+1,2*j,1],points[2*i+1,2*j,2],points[2*i+1,2*j,3],points[2*i+1,2*j,4]))
                                    objlist.append((points[2*i,2*j+1,0],points[2*i,2*j+1,1],points[2*i,2*j+1,2],points[2*i,2*j+1,3],points[2*i,2*j+1,4]))
                                    objlist.append((points[2*i+1,2*j+1,0],points[2*i+1,2*j+1,1],points[2*i+1,2*j+1,2],points[2*i+1,2*j+1,3],points[2*i+1,2*j+1,4]))
                    
            print(objlist)
                
            print(time.time()-starttime)

    
    
if __name__ == '__main__':
    q=Queue()
    imageProcess=Process(target=retriveDepth,args=(q,))
    filterProcess = Process(target=filterDepth,args=(q,))
    imageProcess.start()
    filterProcess.start()
    imageProcess.join()
    filterProcess.join()
