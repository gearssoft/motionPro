#%%
from ultralytics import YOLO,checks
import cv2 as cv

import os
from struct import *
import pygame
import time
import numpy as np

from dotenv import load_dotenv
from rs2_device_manager import Rs2DeviceManagers,dummyDeviceManager,RS2_getCloud

# from udpThread import udpServerThread as udpServer
from tcpThread import tcpServerThread as tcpServer

import argparse
import lz4.frame

#%%
_checkcode = 20231026
__version__ = (0,0,1)
#%%
# .env 파일 로드
load_dotenv()

# .env 파일에서 변수 읽기
mask_weights = os.getenv('mask_weights')
pose_weights = os.getenv('pose_weights')
mask_conf_thres = float(os.getenv('mask_conf_thres'))

# 숫자인지 확인후 int로 변환
if os.getenv('mask_device').isdecimal():
    mask_device = int(os.getenv('mask_device'))
else :
    mask_device = os.getenv('mask_device')

if os.getenv('pose_device').isdecimal():
    pose_device = int(os.getenv('pose_device'))
else :
    pose_device = os.getenv('pose_device')

iou_threshold = float(os.getenv('iou_threshold'))


swape_length_threshold = float(os.getenv('swape_length_threshold'))
swape_hoz_threshold = float(os.getenv('swape_hoz_threshold'))
swape_linearity_threshold= float(os.getenv('swape_linearity_threshold'))

screen_width = int(os.getenv('screen_width'))
screen_height = int(os.getenv('screen_height'))

# pose_data_port = int(os.getenv('pose_data_port'))
# segment_data_port = int(os.getenv('segment_data_port'))
# target_ip = os.getenv('target_ip')

#%%
current_color_frame = None
current_depth_frame = None

#%%커멘드라인 인자 
## --mode [server|test] , --camera [camera_index]

parser = argparse.ArgumentParser(description='bodyTracker')
parser.add_argument('--mode', type=str, default='test', help='server or test')
parser.add_argument('--camera', type=str, default=os.getenv('camera'), help='camera serial number')
parser.add_argument('--port', type=int, default=int(os.getenv('port')), help='port')

#카메라가 -1 이아니라면 인자로 받은 카메라 인덱스로 설정
# if parser.parse_args().camera >= 0:
#     camera_Serial = parser.parse_args().camera

__ApplicationRunMode = parser.parse_args().mode

camera_Serial = parser.parse_args().camera
port = parser.parse_args().port

#%%
mask_model = YOLO(mask_weights)  # load a pretrained YOLOv5s detection model

print(f'model loaded success : {mask_model}')
print('model names : ',mask_model.names)

#warmup
warm_up_img = np.zeros((screen_width, screen_height, 3), dtype=np.uint8)  # Creating a blank image with dimensions 416x416
print('Warming up the model...')
time.sleep(0.5)
mask_model(warm_up_img)  # Performing a warm-up inference
time.sleep(0.5) # Wait for the model to complete the warm-up
print(f'Model warmed up. ok')

#%%
remote_target = None

def makePacket(mask_results,frame,_depth_frame,camera_index):
    _packet = None
    for mask_result in mask_results:
        detectionCount = len(mask_result.boxes)

        if detectionCount > 0:   

            mask_datas = mask_result.masks.data.cpu().numpy()

            _, _, channels = frame.shape

            combined_mask = np.zeros_like(mask_datas[0], dtype=np.uint8)
            for mask_data in mask_datas:        
                mask_img = np.where(mask_data > 0.5, 255, 0).astype('uint8')  # 0.5 이상인 값을 255로 바꾸고, uint8로 변경
                combined_mask = np.bitwise_or(combined_mask, mask_img)  # 마스크 이미지 합성

            alpha_channel = combined_mask  # 알파 채널로 이진 마스크 사용
            
            if channels < 4:
                rgba_mask = cv.cvtColor(combined_mask, cv.COLOR_GRAY2RGBA)  # GRAY to RGBA
                rgba_mask[:, :, 3] = alpha_channel  # 알파 채널 설정
            else:
                rgba_mask = combined_mask.copy()
                rgba_mask[:, :, 3] = alpha_channel  # 알파 채널 설정

            # 원본 이미지를 RGBA로 변환            
            if channels < 4:
                rgba_image = cv.cvtColor(frame, cv.COLOR_BGR2BGRA)
            else:
                rgba_image = frame.copy()
                
            # 원본 이미지의 차원에 맞게 마스크 이미지 리사이즈
            rgba_mask_resized = cv.resize(rgba_mask, (rgba_image.shape[1], rgba_image.shape[0]), interpolation=cv.INTER_AREA)

            # 리사이즈된 마스크 이미지와 원본 이미지 합성
            masked_image = cv.bitwise_and(rgba_image, rgba_mask_resized)
            _width,_height =  masked_image.shape[1],masked_image.shape[0]

            boxes = mask_result.boxes.xyxyn.cpu().numpy()

            _header_packet = pack('<LBBBB', _checkcode, 0x16,0,detectionCount,camera_index)
            
            _packet = b''
            
            #intrinsics
            depth_intrinsics = _depth_frame.profile.as_video_stream_profile().intrinsics  
            
            _packet += pack('<L',depth_intrinsics.width)
            _packet += pack('<L',depth_intrinsics.height)
            _packet += pack('<fffff',*depth_intrinsics.coeffs)
            _packet += pack('<ffff',depth_intrinsics.fx,depth_intrinsics.fy,depth_intrinsics.ppx,depth_intrinsics.ppy)
            
            for idx, box in enumerate(boxes): 
                #box               
                _packet += pack('<ffff', *box)                
                
                #center point
                x1,y1,x2,y2 = map(int,box*[_width,_height,_width,_height])

                center_x = int((x1+x2)/2)
                center_y = int((y1+y2)/2)
                # #get depth
                depth = _depth_frame.get_distance(center_x,center_y)
                _packet += pack('<fff',center_x,center_y,depth)

                # #write segmentation image
                cropped_image = masked_image[y1:y2, x1:x2]
                #  # 이미지를 PNG로 압축합니다.
                _, png_bytes = cv.imencode('.png', cropped_image)                

                # # 패킷에 이미지 크기와 이미지 데이터를 추가합니다.
                _packet += pack('<L', len(png_bytes))  # 이미지 크기
                _packet += png_bytes.tobytes()  # 이미지 데이터
                
                pointCloud,rgbs = RS2_getCloud(_depth_frame,masked_image,x1,y1,x2-x1,y2-y1,downsample_factor=8)
                              
                pointCloudByte = pointCloud.tobytes()
                _packet += pack('<L',len(pointCloudByte))
                _packet += pointCloudByte
                
                rgbsByte = rgbs.tobytes()
                _packet += pack('<L',len(rgbsByte))
                _packet += rgbsByte
                

            _packet = _header_packet + pack('<L',len(_packet)) + _packet


    return _packet
     
def detect_proc(_depth_frame,_color_frame):

    frame = _color_frame 
    # 추론
    mask_results = mask_model(frame, conf=mask_conf_thres,device=pose_device,verbose=False,classes=[0],iou = iou_threshold)     

    mask_packet = makePacket(mask_results,frame,_depth_frame,0)
    
    return mask_packet

def server_task(depth_cam):   

    def onTcpPacket(conn,_data, rinfo):
        global remote_target

        checkcode,cmd,*param = unpack("<LBBBB",_data[:8])
        
        if checkcode != _checkcode :
            print(f'checkcode error : {checkcode}')
            return
        
        _packet = None
        
        if cmd == 0x10 :
            print(f'ping from {rinfo}')
            _packet = pack('<LBBBB',checkcode,cmd,*__version__)
            timestamp = int(time.time()) # Get the current Unix timestamp
            _packet += pack('<L', timestamp) # Add the timestamp to the packet
            _packet += bytearray(4) # Leave 4 bytes for future use
        elif cmd == 0x14: # req set remote target
            remote_target = conn
        elif cmd == 0x15: # req set remote target
            remote_target = None    
                    
        if _packet is not None:
            conn.sendall(_packet)
    
    def onConnect(conn,rinfo):
        global remote_target
        remote_target = conn

    def onClose(conn,rinfo):
        global remote_target
        remote_target = None
        print(f'client disconnected : {rinfo}')
        pass

    _tcpServer = tcpServer(port,onTcpPacket,onClose,onConnect)
    _tcpServer.start()   

    print('to stop press Ctrl+C')    
    
    try :        
        while True:
            
            #camera frame
            _depth_frame,_color_frame = depth_cam.get_frames()
            frame = np.asanyarray(_color_frame.get_data())

            if remote_target is not None:
                try:
                    _packet = detect_proc(_depth_frame, frame)
                    
                    
                    
                    
                    if _packet is not None:                        
                        remote_target.sendall(_packet)                        
                except ConnectionResetError as e:
                    print(f"Connection reset by peer: {e}")
                except BrokenPipeError as e:
                    print(f"Broken pipe error: {e}")
                except Exception as e:
                    print(f"An unexpected error occurred: {e}")
                
    except KeyboardInterrupt:        
        print('KeyboardInterrupt')        
    except Exception as e:
        print(e)
    _tcpServer.terminate()
    print('server task end')
            
# main 검사
if __name__ == '__main__':
    
    print(f'opencv version {cv.__version__}')
    checks()
    try :
        #camera Serial 앞에 image_ 가 붙으면 이미지 파일을 읽어서 재생합니다.
        if camera_Serial[:6] != 'image_':
            print('\nInit Realsense2 camera...\n\n')
            dm = Rs2DeviceManagers()        
            depth_cam = dm.get_device_managerBySerialNumber(camera_Serial)
            if depth_cam is None:
                print(f'camera not found : {camera_Serial}')
                exit()
            depth_cam.start()
        else :            
            _img_file = camera_Serial[6:]
            depth_cam = dummyDeviceManager(imagefile=_img_file)
            depth_cam.start()
            
        server_task(depth_cam)
        
        depth_cam.stop()
        print('Done.')
    except Exception as e:
        print(e)        
        print('Exit.')  