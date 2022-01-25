import camera
import segment
import _pickle as pickle
import numpy as np
import cv2
import Color as c
import ctypes
from numpy.ctypeslib import ndpointer 

 # Or full path to file 
# Now whenever argument
# will be passed to the function                                                       
# ctypes will check it.

class Object():
    def __init__(self, x = -1, y = -1, size = -1, distance = -1, exists = False):
        self.x = x
        self.y = y
        self.size = size
        self.distance = distance
        self.exists = exists     

    def __str__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)

    def __repr__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)


# results object of image processing. contains coordinates of objects and frame data used for these results
class ProcessedResults():

    def __init__(self, 
                balls=[], 
                basket_b = Object(exists = False), 
                basket_m = Object(exists = False), 
                color_frame = [],
                depth_frame = [],
                fragmented = [],
                debug_frame = [],
                is_obstacle_close = False) -> None:


        self.balls = balls
        self.basket_b = basket_b
        self.basket_m = basket_m
        self.color_frame = color_frame
        self.depth_frame = depth_frame
        self.fragmented = fragmented
        self.is_obstacle_close = is_obstacle_close
        # can be used to illustrate things in a separate frame buffer
        self.debug_frame = debug_frame


#Main processor class. processes segmented information
class ImageProcessor():
    def __init__(self, camera, color_config = "colors/colors.pkl", debug = False):
        self.camera = camera

        self.color_config = color_config
        with open(self.color_config, 'rb') as conf:
            self.colors_lookup = pickle.load(conf)
            self.set_segmentation_table(self.colors_lookup)

        self.fragmented	= np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

        self.t_balls = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_basket_b = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_basket_m = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

        self.debug = debug
        self.debug_frame = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        
        self.utils = ctypes.CDLL("./utils.so")
        self.utils.processBorders.argtypes = [ndpointer(np.uint8, flags="C_CONTIGUOUS"),
                ctypes.c_size_t,
                ndpointer(np.uint32, flags="C_CONTIGUOUS"),
                ctypes.c_size_t]
        self.utils.processBorders.restype = None
        #self.utils = ctypes.CDLL("./utils.so")
        self.utils.isObstacle.argtypes = [ndpointer(np.uint8, flags="C_CONTIGUOUS"),
                ctypes.c_size_t]
        self.utils.isObstacle.restype = np.int8

    def set_segmentation_table(self, table):
        segment.set_table(table)

    def start(self):
        self.camera.open()

    def stop(self):
        self.camera.close()

    def analyze_balls(self, t_balls, fragments) -> list:
        contours, hierarchy = cv2.findContours(t_balls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        balls = []
        balls_array = np.empty((0,4),dtype=np.uint32)
        for contour in contours:

            # ball filtering logic goes here. Example includes filtering by size and an example how to get pixels from
            # the bottom center of the fram to the ball

            size = int(cv2.contourArea(contour))

            if size < 15:
                continue

            x, y, w, h = cv2.boundingRect(contour)


            ys	= np.arange(y + h, self.camera.rgb_height)
            xs	= np.linspace(x + w//2, self.camera.rgb_width // 2, num=len(ys))
            #line_array = fragments[ys[0], xs[0]]
            #print(xs[0], ys[0])
            #print(fragments)


            obj_x = int(x + (w/2))
            obj_y = int(y + (h))
            #if self.debug:
                    #self.debug_frame[ys, xs] = [0, 0, 0]
                    #cv2.circle(self.debug_frame,(obj_x, obj_y), 10, (0,255,0), 2)
                    #print("Hello World")
            #obj_dst = obj_y*obj_y + (obj_x - self.camera.rgb_width //2)*(obj_x - self.camera.rgb_width //2) #Remove math              
            balls_array = np.append(balls_array, np.array([[np.uint32(obj_x), np.uint32(obj_y), np.uint32(size), np.uint32(True)]]), axis=0)       
        #print(balls)
        #print(balls_array.astype(np.uint8))
        #print(fragments.astype(np.uint8))
        #balls_array = balls_array.astype(np.uint8)
        self.utils.processBorders(fragments, fragments.size, balls_array, balls_array.size)
        #print(balls_array)
        for ball in balls_array:            
            if(ball[3] == 1):
                #print(ball)
                balls.append(Object(x = int(ball[0]), y = int(ball[1]), size = int(ball[2]),
                    distance = int((self.camera.rgb_height - ball[1])*(self.camera.rgb_width - ball[1]) + (ball[0] - self.camera.rgb_width //2)*(ball[0] - self.camera.rgb_width //2)*0.2),
                    exists = True))
                if self.debug:
                    #self.debug_frame[ys, xs] = [0, 0, 0]
                    cv2.circle(self.debug_frame,(ball[0], ball[1]), 10, (0,255,0), 2)
                    #print("Hello World")
        balls.sort(key= lambda x: x.distance) #distance is inversed!!!
        #print(balls)
        #print(fragments)
        '''
        color_image = np.asanyarray(color_frame.get_data(),ctypes.c_int)
        
        
        print(balls)
        balls = [x for x in balls if x.exists]
        print(fragments)
        print(balls)
        for ball in balls:
            if self.debug:
                #self.debug_frame[ys, xs] = [0, 0, 0]
                cv2.circle(self.debug_frame,(obj_x, obj_y), 10, (0,255,0), 2)
                #print("Hello World")
        '''   
        
        return balls

    def analyze_baskets(self, t_basket, depth_frame, debug_color = (0, 255, 255)) -> list:
        contours, hierarchy = cv2.findContours(t_basket, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        baskets = []
        for contour in contours:

            # basket filtering logic goes here. Example includes size filtering of the basket

            size = cv2.contourArea(contour)

            if size < 100:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            obj_dst = None
            if(obj_x is not None):
                obj_dst = depth_frame[obj_y-1,obj_x-1]*self.camera.depth_scale

            baskets.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

        baskets.sort(key= lambda x: -x.distance)

        basket = next(iter(baskets), Object(exists = False))

        if self.debug:
            if basket.exists:
                cv2.circle(self.debug_frame,(basket.x, basket.y), 20, debug_color, -1)

        return basket
        
    def analyze_obstacle(self, fragments):
        
        
        return 

    def get_frame_data(self, aligned_depth = False):
        if self.camera.has_depth_capability():
            return self.camera.get_frames(aligned = aligned_depth)
        else:
            return self.camera.get_color_frame(), np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

    def process_frame(self, aligned_depth = False) -> ProcessedResults:
        color_frame, depth_frame = self.get_frame_data(aligned_depth = aligned_depth)

        segment.segment(color_frame, self.fragmented, self.t_balls, self.t_basket_m, self.t_basket_b)

        if self.debug:
            self.debug_frame = np.copy(color_frame)

        balls = self.analyze_balls(self.t_balls, self.fragmented)
        basket_b = self.analyze_baskets(self.t_basket_b, depth_frame, debug_color=c.Color.BLUE.color.tolist())
        basket_m = self.analyze_baskets(self.t_basket_m, depth_frame, debug_color=c.Color.MAGENTA.color.tolist())
        is_obstacle_close = int(self.utils.isObstacle(self.fragmented, self.fragmented.size))
        return ProcessedResults(balls = balls, 
                                basket_b = basket_b, 
                                basket_m = basket_m, 
                                color_frame=color_frame, 
                                depth_frame=depth_frame, 
                                fragmented=self.fragmented, 
                                debug_frame=self.debug_frame,
                                is_obstacle_close = is_obstacle_close)
