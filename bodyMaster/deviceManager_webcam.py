import numpy as np
import cv2 as cv

class ColoFrame:
    def __init__(self,img):
        self.img = img

    def get_data(self):
        return self.img
    
class webcamDeviceManager:
    def __init__(self,camera_id=0,frame_width=640,frame_height=480):
        cap = self.cap = cv.VideoCapture(0)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.frame = None
        
    def start(self):
        pass
    def stop(self):
        pass
    def get_frames(self):
        ret,frame = self.cap.read()
        
        if ret == False:
            return None,None
        
        return None,ColoFrame(frame)