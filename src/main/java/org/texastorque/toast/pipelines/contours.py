import time
import cv2
import math
import sys
import intrinsics
from geometry import *


def update(frame_r, i_vals, net):
    frame_g = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
    
    net.store_frame(frame_r)
    return frame_r

# Required function for pipeline
def loop(camera_name, cam, i_vals, net):

    while True:
        ret, frame_r = cam.read()
        if not ret:
            print("Not working")
            break

        update(frame_r, i_vals, net)

    cam.release()
    cv2.destroyAllWindows()
