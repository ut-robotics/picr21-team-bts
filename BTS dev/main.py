#B_T_S Final Code#
#January 26th 2022#

import image_processor
import camera
import cv2

import time

cam = camera.RealsenseCamera(rgb_height=540, rgb_width=960,rgb_framerate=60)
processor = image_processor.ImageProcessor(cam, debug=True)

processor.start()

start = time.time()
fps = 0
frame = 0
frame_cnt = 0

try:
    while True:
        # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
        processedData = processor.process_frame()
        print(len(processedData.balls))
        frame_cnt +=1

        frame += 1
        if frame % 30 == 0:
            frame = 0
            end = time.time()
            fps = 30 / (end - start)
            start = end
            print("FPS: {}, framecount: {}".format(fps, frame_cnt))

        debug_frame = processedData.debug_frame

        cv2.imshow('debug', debug_frame)

        k = cv2.waitKey(1) & 0xff
        if k == ord('q'):
            break
except KeyboardInterrupt:
    print("closing....")
finally:
    cv2.destroyAllWindows()
    processor.stop()
