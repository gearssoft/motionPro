#%%
from ultralytics import YOLO,checks
import cv2 as cv

import os
from struct import *
import pygame
import time
import numpy as np

from dotenv import load_dotenv
# from horse_house.motionPro.bodyMaster.deviceManager_rs2 import Rs2DeviceManagers,dummyDeviceManager


import argparse
import socket

#from multiprocessing import Queue,Process

from tasks import server_task,rendering_task

checks()

#%%

__version__ = (0,0,4)
#%%
# .env 파일 로드
load_dotenv(".env")

# .env 파일에서 변수 읽기
mask_weights = os.getenv('mask_weights')
pose_weights = os.getenv('pose_weights')
mask_conf_thres = float(os.getenv('mask_conf_thres'))




#%%
current_color_frame = None
current_depth_frame = None

#%%커멘드라인 인자 
## --mode [server|test] , --camera [camera_index]

parser = argparse.ArgumentParser(description='bodyTracker')
parser.add_argument('--mode', type=str, help='server or test')
parser.add_argument('--camera', type=str, help='camera serial number')
parser.add_argument('--port', type=int, help='port')
parser.add_argument('--debug_port', type=int, default=0, help='debug port')

#카메라가 -1 이아니라면 인자로 받은 카메라 인덱스로 설정
# if parser.parse_args().camera >= 0:
#     camera_Serial = parser.parse_args().camera

__ApplicationRunMode = parser.parse_args().mode

camera_Serial = parser.parse_args().camera
port = parser.parse_args().port
debug_port = parser.parse_args().debug_port

#%%
#warmup
screen_width = int(os.getenv('screen_width'))
screen_height = int(os.getenv('screen_height'))
send_data_type = os.getenv('send_data_type')

warm_up_img = np.zeros((screen_width, screen_height, 3), dtype=np.uint8)  # Creating a blank image with dimensions 416x416

pose_model = None

mask_model = None


if send_data_type == 'pose' or send_data_type == 'all':
    pose_model = YOLO(pose_weights)  # load a pretrained YOLOv5s detection model
    print('##Warming up the pose model...')
    time.sleep(0.5)
    pose_model(warm_up_img)  # Performing a warm-up inference
if send_data_type == 'mask' or send_data_type == 'all':
    mask_model = YOLO(mask_weights)  # load a pretrained YOLOv5s detection model
    print('##Warming up the mask model...')
    time.sleep(0.5)
    mask_model(warm_up_img)  # Performing a warm-up inference

time.sleep(1.0) # Wait for the model to complete the warm-up
print(f'##Model warmed up. ok')

#%%

process_msg_socket = None
if debug_port > 0:
    process_msg_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    process_msg_server_address = ('localhost', debug_port)    

def send_process_msg(msg):
    if process_msg_socket is not None:
        process_msg_socket.sendto(msg.encode(), process_msg_server_address)

try :
    
    send_process_msg(f'start system version : {__version__} ')

    #camera Serial 앞에 image_ 가 붙으면 더미 카메라 모드 , 이미지 파일을 읽어서 재생합니다.
    if camera_Serial[:6] == 'image_':
        
        from deviceManager_rs2 import dummyDeviceManager
        
        _img_file = camera_Serial[6:]
        cam_device = dummyDeviceManager(imagefile=_img_file)
        cam_device.start()
        time.sleep(1)
        send_process_msg('dummy camera ok')
    elif camera_Serial[:7] == 'webcam:':
        camidx = int(camera_Serial[7:])
        from deviceManager_webcam import webcamDeviceManager
        cam_device = webcamDeviceManager(camera_id=camidx,frame_width=screen_width,frame_height=screen_height)
        
    else :            
        send_process_msg('init camera')
        print('\n##Init Realsense2 camera...\n\n')
        
        while True:        
            try :
                from deviceManager_rs2 import Rs2DeviceManagers
                
                dm = Rs2DeviceManagers()  
                
                cam_device = dm.get_device_managerBySerialNumber(camera_Serial)
                if cam_device is None:
                    print(f'##camera not found : {camera_Serial}')
                    send_process_msg(f'##camera not found : {camera_Serial}')
                    time.sleep(3)
                    continue
                
                print(f'##camera found : {camera_Serial}')
                cam_device.start()
                break
                
            except Exception as e:
                print(e)
                print('##retry...')
                time.sleep(3)
        
        print("camera init ok");
        

        send_process_msg('camera ok')
    
    send_process_msg('start process')
    if __ApplicationRunMode == 'server':        
        # server_task(cam_device)
        server_task(cam_device,port=port,version_info=__version__,pose_model=pose_model,send_process_msg=send_process_msg)
    else :        
        rendering_task(cam_device,pose_model=pose_model)
        
    print(f"app start : {__ApplicationRunMode}")
    print("to quit press ESC or close the window")

    cam_device.stop()
    print('###Done.')
    send_process_msg('end process')
except Exception as e:
    print(e)        
    print('###Exit.')  
    send_process_msg('exit process' + str(e))
finally:
    if process_msg_socket is not None:
        process_msg_socket.close()
        