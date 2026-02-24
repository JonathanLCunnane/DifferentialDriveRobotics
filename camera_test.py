import brickpi3
import time
import cv2 
import numpy as np
from picamera2 import Picamera2
 
BP = brickpi3.BrickPi3()
 
 
 
 
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
 
picam2.start()
 
starttime = time.time()
 
 
for i in range(1000):
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


    
    cv2.imwrite("test_camera.jpg", img)
    print("drawImg:" + "/home/pi/prac-files/test_camera.jpg")
    print("Captured image", i, "at time", time.time() - starttime)
 
picam2.stop()