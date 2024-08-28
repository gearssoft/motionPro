#%%
from ultralytics import YOLO,checks
import cv2 as cv

import os
from struct import *
import pygame
import time
import numpy as np
# from dotenv import load_dotenv
import threading
import queue

from sklearn.linear_model import LinearRegression
from scipy.spatial import distance

from dotenv import load_dotenv
from rs2_device_manager import Rs2DeviceManagers,dummyDeviceManager


from myGeoUtils import draw_detection,draw_crosshair
from udpThread import udpServerThread as udpServer

import argparse
from cliThread import CLIThread

#%%
_checkcode = 20231026
__version__ = (0,0,1)
#%%
# .env 파일 로드
load_dotenv()

# .env 파일에서 변수 읽기
pose_weight = os.getenv('pose_weight')
mask_weights = os.getenv('mask_weights')
mask_conf_thres = float(os.getenv('mask_conf_thres'))
pose_conf_thres = float(os.getenv('pose_conf_thres'))

# 숫자인지 확인후 int로 변환
if os.getenv('mask_device').isdecimal():
    mask_device = int(os.getenv('mask_device'))
else :
    mask_device = os.getenv('mask_device')

if os.getenv('pose_device').isdecimal():
    pose_device = int(os.getenv('pose_device'))
else :
    pose_device = os.getenv('pose_device')

camera_index = int(os.getenv('camera'))

swape_length_threshold = float(os.getenv('swape_length_threshold'))
swape_hoz_threshold = float(os.getenv('swape_hoz_threshold'))
swape_linearity_threshold= float(os.getenv('swape_linearity_threshold'))

screen_width = int(os.getenv('screen_width'))
screen_height = int(os.getenv('screen_height'))

pose_data_port = int(os.getenv('pose_data_port'))
target_ip = os.getenv('target_ip')

#%%
current_color_frame = None
current_depth_frame = None

#%%커멘드라인 인자 
## --mode [server|test] , --camera [camera_index]

parser = argparse.ArgumentParser(description='bodyTracker')
parser.add_argument('--mode', type=str, default='test', help='server or test')
parser.add_argument('--camera', type=int, default=-1, help='camera index')

#카메라가 -1 이아니라면 인자로 받은 카메라 인덱스로 설정
if parser.parse_args().camera != -1:
    camera_index = parser.parse_args().camera

__ApplicationRunMode = parser.parse_args().mode


#%%
pose_model = YOLO(pose_weight)  # load a pretrained YOLOv8n detection model
mask_model = YOLO(mask_weights)  # load a pretrained YOLOv5s detection model

print(f'model loaded success : {mask_model}')
print('model names : ',mask_model.names)
# Load class names
class_names = mask_model.names

# msg_queue.put('model loaded success')

#warmup
warm_up_img = np.zeros((screen_width, screen_height, 3), dtype=np.uint8)  # Creating a blank image with dimensions 416x416
print('Warming up the model...')
time.sleep(0.5)

pose_model(warm_up_img)  # Performing a warm-up inference
mask_model(warm_up_img)  # Performing a warm-up inference
time.sleep(1.0) # Wait for the model to complete the warm-up
print(f'Model warmed up. ok')



#%%
detections_repogitory = {}

def remove_stale_objects(detection_repo, max_age=3):
    """
    Remove stale objects from the detection repository.

    Args:
        detection_repo (dict): The detection repository.
        max_age (float): The maximum age of objects (in seconds) to retain.

    Returns:
        None
    """
    current_time = time.time()  # Get the current time

    # Get the list of keys that have stale objects
    stale_keys = [key for key, value in detection_repo.items() if current_time - value[-1] > max_age]

    # Remove the stale objects from the repository
    for key in stale_keys:
        del detection_repo[key]

def pose_proc(depth_cam):
    global current_color_frame,current_depth_frame
    _depth_frame,_color_frame = depth_cam.get_frames()
    
    # frame = cv.flip(np.asanyarray(_color_frame.get_data()),1) # Flip the frame horizontally
    frame = np.asanyarray(_color_frame.get_data())
    
    # 추론
    pose_results = pose_model.track(frame, conf=pose_conf_thres,device=pose_device,verbose=False)        
    current_color_frame = frame
    current_depth_frame = _depth_frame
    
    if len(pose_results) > 0:
        for result in pose_results:
            
            if result.boxes.id == None :
                continue      
                        
            ids = result.boxes.id.cpu().numpy().astype(int) 
            boxes = result.boxes.xyxyn.cpu().numpy()
            keypoints = result.keypoints.xyn.cpu().numpy()
            
            for idx,np_keypoints in enumerate(keypoints):
                box = boxes[idx]
                _id = ids[idx]
                x1,y1,x2,y2 = map(int,box * [screen_width, screen_height, screen_width, screen_height])   
                # np_keypoints = keypoints[idx] 
                if np_keypoints.shape[0] != 17:
                    continue         
                
                kpt_head = np_keypoints[0]
                
                kpt_lear = np_keypoints[3]
                kpt_lhand = np_keypoints[9]
                kpt_lelbow = np_keypoints[7]
                kpt_lshoulder = np_keypoints[5]

                kpt_rear = np_keypoints[4]
                kpt_rhand = np_keypoints[10]
                kpt_relbow = np_keypoints[8]
                kpt_rshoulder = np_keypoints[6]
                
                # Check if any of the keypoints is out of bounds
                keypoints = [kpt_head,kpt_lhand,kpt_lshoulder,kpt_rhand,kpt_rshoulder] # depth 에 쓰일부위만 따로 모아서 체크
                # keypoints = []
                skip = any(
                    (kpt[0] <= 0 and kpt[1] <= 0) or
                    (kpt[0] >= 1 or kpt[1] >= 1)
                    for kpt in keypoints
                )

                if skip:
                    continue  # Skip processing for this box if any keypoint is out of bounds
                
                #key point 출력
                r_ear_posx, r_ear_posy = map(int, (kpt_rear[0]*screen_width, kpt_rear[1]*screen_height))
                l_ear_posx, l_ear_posy = map(int, (kpt_lear[0]*screen_width, kpt_lear[1]*screen_height))

                r_hand_posx, r_hand_posy = map(int, (kpt_rhand[0]*screen_width, kpt_rhand[1]*screen_height))                
                l_hand_posx, l_hand_posy = map(int, (kpt_lhand[0]*screen_width, kpt_lhand[1]*screen_height))

                r_shoulder_posx, r_shoulder_posy = map(int, (kpt_rshoulder[0]*screen_width, kpt_rshoulder[1]*screen_height))
                l_shoulder_posx, l_shoulder_posy = map(int, (kpt_lshoulder[0]*screen_width, kpt_lshoulder[1]*screen_height))
                
                head_posx, head_posy = map(int, (kpt_head[0]*screen_width, kpt_head[1]*screen_height))                    

                if _depth_frame is not None:
                    #거리측정
                    head_depth = _depth_frame.get_distance( _depth_frame.width - head_posx, head_posy)
                    lhand_depth = _depth_frame.get_distance(_depth_frame.width -l_hand_posx, l_hand_posy)
                    rhand_depth = _depth_frame.get_distance(_depth_frame.width -r_hand_posx, r_hand_posy)
                    lshoulder_depth = _depth_frame.get_distance(_depth_frame.width -l_shoulder_posx, l_shoulder_posy)
                    rshoulder_depth = _depth_frame.get_distance(_depth_frame.width -r_shoulder_posx, r_shoulder_posy)
                else :
                    head_depth = 1.5
                    lhand_depth = 1.5
                    rhand_depth = 1.5
                    lshoulder_depth = 1.5
                    rshoulder_depth = 1.5

                _detection = [_id,box,                              
                              kpt_head,
                              kpt_lear,kpt_rear,
                              kpt_lhand,kpt_rhand,
                              kpt_lelbow,kpt_relbow,
                              kpt_lshoulder,kpt_rshoulder,                              
                              head_depth,
                              lhand_depth,rhand_depth,lshoulder_depth,rshoulder_depth,
                              time.time()      #timestemp
                                    ]
                detections_repogitory[_id] = _detection
        
    # 시간이 너무 오래 경과된 것들 삭제(값이 변하지않는 상태로 3초 이상 경과된 것들)
    remove_stale_objects(detections_repogitory, max_age=3)

    return frame

remote_target = None
def server_task(depth_cam):

    def _sendPoseData(server,rinfo):
        global detections_repogitory
        global _checkcode

        # Sort the detections_repository by the head_depth value
        # Assuming each detection has the head_depth value at index 6
        sorted_detections = sorted(
            detections_repogitory.values(),
            key=lambda detection: detection[12]
        )[:4]  # Select the closest 8 detections

        # Prepare the header of the packet
        _packet = pack('<LBBBB', _checkcode, 0x11,0,len(sorted_detections),0)

        # Pack the data of each detection into the packet
        for detection in sorted_detections:
            # Assuming each value in the detection is a float
            # Adjust this loop according to the actual structure of your detections
            for value in detection:
                if isinstance(value, tuple):  # If the value is a tuple, pack each element of the tuple
                    _packet += pack(f'<{len(value)}f', *value)
                elif isinstance(value, np.ndarray):  # If the value is a numpy array
                    _packet += pack(f'<{len(value)}f', *value.tolist())
                elif isinstance(value, np.int32):  # If the value is an integer
                    _packet += pack('<L', value)
                else:
                    _packet += pack('<f', value)
        
        # Send the packet to the client
        server.send(_packet, rinfo)
    def _send3DPoseData(server,rinfo):
        global detections_repogitory
        global _checkcode

        sorted_detections = sorted(
            detections_repogitory.values(),
            key=lambda detection: detection[12] #sort by head_depth
        )[:4]  # Select the closest 4 detections

        # Prepare the header of the packet
        _packet = pack('<LBBBB', _checkcode, 0x13,0,len(sorted_detections),0)

        for detection in sorted_detections:

            kpt_head =  list(map(int,detection[2]* (current_depth_frame.width, current_depth_frame.height)))
            kpt_lhand = list(map(int,detection[5]* (current_depth_frame.width, current_depth_frame.height)))
            kpt_rhand = list(map(int,detection[6]* (current_depth_frame.width, current_depth_frame.height)))
            kpt_lshoulder = list(map(int,detection[9]* (current_depth_frame.width, current_depth_frame.height)))
            kpt_rshoulder = list(map(int,detection[10]* (current_depth_frame.width, current_depth_frame.height)))

            #get point 3d
            kpt3d_head = depth_cam.get_3d_coordinates(kpt_head[0],kpt_head[1])
            kpt3d_lhand = depth_cam.get_3d_coordinates(kpt_lhand[0],kpt_lhand[1])
            kpt3d_rhand = depth_cam.get_3d_coordinates(kpt_rhand[0],kpt_rhand[1])
            kpt3d_lshoulder = depth_cam.get_3d_coordinates(kpt_lshoulder[0],kpt_lshoulder[1])
            kpt3d_rshoulder = depth_cam.get_3d_coordinates(kpt_rshoulder[0],kpt_rshoulder[1])

            _packet += pack('<L3f3f3f3f3f',detection[0],*kpt3d_head,*kpt3d_lhand,*kpt3d_rhand,*kpt3d_lshoulder,*kpt3d_rshoulder)
        server.send(_packet, rinfo)
    def _sendFaceImageData(server,rinfo,_detection):        
        x1, y1, x2, y2 = _detection[1]

        learx,leary = _detection[3]
        rearx,reary = _detection[4]

        lshoulderx,lshouldery = _detection[9]
        rshoulderx,rshouldery = _detection[10]

        # x1 = min(learx,rearx)
        
        # x2 = max(learx,rearx)
        y2 = lshouldery

        img_w, img_h = current_color_frame.shape[1], current_color_frame.shape[0]

        x1, y1, x2, y2 = map(int, (x1 * img_w, y1 * img_h, x2 * img_w, y2 * img_h))

        _face_img = current_color_frame[y1:y2, x1:x2]

        mask_results = mask_model(_face_img, conf=mask_conf_thres, device=mask_device,classes=0)
        
        if len(mask_results) > 0:
            print(mask_results[0])
            mask_result = mask_results[0]
            _mask_data = mask_result.masks.data[0]
            mask_img = _mask_data.cpu().numpy()
            mask_img = np.where(mask_img > 0.5, 255, 0).astype('uint8')  # 0.5 이상인 값을 255로 바꾸고, uint8로 변경

            alpha_channel = mask_img  # 알파 채널로 이진 마스크 사용
            _, _, channels = _face_img.shape
            if channels < 4:
                rgba_mask = cv.cvtColor(mask_img, cv.COLOR_GRAY2RGBA)  # GRAY to RGBA
                rgba_mask[:, :, 3] = alpha_channel  # 알파 채널 설정
            else:
                rgba_mask = mask_img.copy()
                rgba_mask[:, :, 3] = alpha_channel  # 알파 채널 설정

            # 원본 이미지를 RGBA로 변환
            _, _, channels = _face_img.shape
            if channels < 4:
                rgba_image = cv.cvtColor(_face_img, cv.COLOR_BGR2RGBA)
            else:
                rgba_image = _face_img.copy()
                
            # 원본 이미지의 차원에 맞게 마스크 이미지 리사이즈
            rgba_mask_resized = cv.resize(rgba_mask, (rgba_image.shape[1], rgba_image.shape[0]), interpolation=cv.INTER_AREA)

            # 리사이즈된 마스크 이미지와 원본 이미지 합성
            _face_img = cv.bitwise_and(rgba_image, rgba_mask_resized)

        print(f'found id {_id} , {_detection}')

        # 이미지를 바이트 배열로 인코딩
        ret, buffer = cv.imencode('.png', _face_img)
        if ret:
            data_size = len(buffer)
            _packet = pack('<LBBBB', checkcode, cmd, 0, _id, 0)
            _packet += pack('<L', data_size)
            _packet += buffer.tobytes()  # 이미지 데이터를 패킷에 추가

            server.send(_packet, rinfo)
        else:
            print("Image encoding failed")
    
    def _sendDetectionData(server,rinfo,_detection):
        global detections_repogitory
        global _checkcode
        _packet = pack('<LBBBB', _checkcode, 0x16,0,len(detections_repogitory),0)


    def onPacket(server,data,rinfo):    
        global target_ip
        global model,_cameraObj
        global remote_target
        
        checkcode,cmd,*param = unpack("<LBBBB",data[:8])

        if checkcode == 10054 :
            print('client disconnected')
            remote_target = None
            return
        
        if checkcode != _checkcode :
            print(f'checkcode error : {checkcode}')
            return
        
        # print(f"checkcode = {checkcode} , cmd = {cmd} , param = {param}")
        
        if cmd == 0x10 : #ping
            print(f'ping from {rinfo}')
            _packet = pack('<LBBBB',checkcode,cmd,*__version__) # 8 byte
            
            timestamp = int(time.time()) # Get the current Unix timestamp
            _packet += pack('<L', timestamp) # Add the timestamp to the packet
            _packet += bytearray(4) # Leave 4 bytes for future use
            
            server.send(_packet,rinfo)
        elif cmd == 0x11: #req all pose                    
            _sendPoseData(server,rinfo)
        elif cmd == 0x12:  # req face image
            _id = param[0]
            print(f'req face image from {rinfo}')
            if _id in detections_repogitory:
                _detection = detections_repogitory[_id]
                _sendFaceImageData(server,rinfo,_detection)                
            else :
                print(f'not found id {_id}')
                _packet = pack('<LBBBB',checkcode,cmd,1,0,0)
                server.send(_packet,rinfo)
                return
        elif cmd == 0x13: # req 3d point
            _send3DPoseData(server,rinfo)
        elif cmd == 0x14: # req set remote target 
            remote_target = rinfo
        elif cmd == 0x15: # req set remote target
            remote_target = None

    _udpServer = udpServer(pose_data_port,onPacket) # udpServer(port,_onUpdate)
    _udpServer.start()             
    
    try :        
        while True:
            pose_proc(depth_cam)
            if remote_target is not None:
                try:
                    # _sendPoseData(_udpServer,remote_target)
                    _send3DPoseData(_udpServer,remote_target)
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

    _udpServer.terminate()
    print('server task end')    
    
def rendering_task(depth_cam):
    # pygame 초기화
    pygame.init()
    
    # 색상 설정
    black = (0, 0, 0)
    white = (255, 255, 255)
    green = (0, 255, 0)

    # 화면 설정
    # screen_width, screen_height = frame_width, frame_height
    screen = pygame.display.set_mode((screen_width, screen_height))
    # font 설정
    msg_font = pygame.font.Font(None, 24)
    last_time = time.time()  # Initialize last_time before entering the loop        
    
    print(f"app start : {__ApplicationRunMode}")
    print("to quit press ESC or close the window")
    bStopRenderTask = False
    while bStopRenderTask == False:

        frame = pose_proc(depth_cam)

        #draw all
        # Convert the frame from BGR to RGB (as OpenCV uses BGR and Pygame uses RGB)
        # Convert the NumPy array to a Pygame surface
        frame_surface = pygame.surfarray.make_surface(cv.flip(np.rot90(cv.cvtColor(frame, cv.COLOR_BGR2RGB)),0))        
        # Clear the screen by camera frame
        screen.blit(pygame.transform.scale(frame_surface, (screen_width, screen_height)), (0, 0))

        for _id,detection in detections_repogitory.items():
            draw_detection(msg_font,screen,detection)
        
        
        # Calculate FPS
        current_time = time.time()
        elapsed_time = current_time - last_time
        fps = 1 / elapsed_time if elapsed_time > 0 else 0
        last_time = current_time
        
        # Render FPS
        fps_text = f"FPS: {fps:.2f} , elapsed_time : {elapsed_time:.2f}"
        fps_surface = msg_font.render(fps_text, True, green)
        screen.blit(fps_surface, (10, 25))  # Adjust the position (10, 10) as needed
        
        pygame.display.update()

        # Pygame 이벤트 처리 (종료 등)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                bStopRenderTask = True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    bStopRenderTask = True
    print('rendering_task end')
    
#%% 스레드 생성 및 시작

# main 검사
if __name__ == '__main__':
    
    print(f'opencv version {cv.__version__}')
    checks()

    try :
        if camera_index >= 0 :

            print('\nInit Realsense2 camera...\n\n')

            dm = Rs2DeviceManagers()        
            depth_cam = dm.get_device_manager(camera_index)
            depth_cam.start()
        else :            
            depth_cam = dummyDeviceManager(imagefile='bus.jpg')
            depth_cam.start()

        if __ApplicationRunMode == 'server':
            server_task(depth_cam)
        else :        
            rendering_task(depth_cam)    
        
        depth_cam.stop()
        print('Done.')
    except Exception as e:
        print(e)        
        print('Exit.')  