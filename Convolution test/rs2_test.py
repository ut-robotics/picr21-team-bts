## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import ctypes
from numpy.ctypeslib import ndpointer

NUM = 16  

fun = ctypes.CDLL("./libfun.so") # Or full path to file 
# Now whenever argument
# will be passed to the function                                                       
# ctypes will check it.
           
fun.myFunction.argtypes = [ctypes.c_int]
fun.recolor.argtypes = [ndpointer(ctypes.c_int, flags="C_CONTIGUOUS")]
fun.recolor.restype = None
fun.recolor.argtypes = [ndpointer(ctypes.c_int, flags="C_CONTIGUOUS"),
                ctypes.c_size_t,
                ndpointer(ctypes.c_int, flags="C_CONTIGUOUS")]
color_image = np.ones((480,640,3),ctypes.c_int)
outdata = np.empty((480,640,3),ctypes.c_int)
print(outdata)
#color_image = np.ones((480,640,3),ctypes.c_int)
#print(color_image.ctypes.data_as(ndpointer(ctypes.c_int, flags="C_CONTIGUOUS")))
# now we can call this
# function using instant (fun)
# returnValue is the value
# return by function written in C
# code
returnValue = fun.myFunction(NUM)  
print(returnValue)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

# Start streaming
pipeline.start(config)
conv_filter = np.array(
    [[1, 1, 1],
    [1, -8, 1],
    [1, 1, 1]],
    np.int32
)
try:    
    while True:
        start = time.time()
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data(),ctypes.c_int)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape
        contur_map = np.zeros(depth_colormap_dim)    
        #r_image = color_image[:,:,0]
        #g_image = color_image[:,:,1]
        #b_image = color_image[:,:,2]        
        fun.recolor(color_image, color_image.size, outdata)       
        #view_shape = tuple(np.subtract(g_image.shape, sub_shape) + 1) + sub_shape
        #strides = 2*r_image.strides
        #sub_matrices = np.lib.stride_tricks.as_strided(r_image,view_shape,strides)
        try:            
           
            #r_image_counter = np.einsum('ij,klij->kl',conv_filter,sub_matrices)
            #r_image_counter = np.pad(r_image_counter, ((1, 1),(1,1)),'edge') 
            #g_image_counter = np.einsum('ij,klij->kl',conv_filter,sub_matrices)
            #g_image_counter = np.pad(g_image_counter, ((1, 1),(1,1)),'edge')           
            #b_image_counter = np.einsum('ij,klij->kl',conv_filter,sub_matrices)
            #b_image_counter = np.pad(b_image_counter, ((1, 1),(1,1)),'edge')
            #test = np.dstack((r_image_counter,g_image_counter,b_image_counter))
            #test = np.reshape(test,(r_image_counter.shape[0],r_image_counter.shape[1],3))            
            
            #color_image[color_image>100] = 255        
            #print(np.amax(test))
            end = time.time()
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((outdata.astype(np.uint8),color_image.astype(np.uint8)))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)
        except Exception as e:
            print(e)
       
        seconds = end - start
        fps  = 1 / seconds
        print(fps)
        
finally:

    # Stop streaming
    pipeline.stop() 
