#%%
from ultralytics import YOLO,checks
import cv2 as cv

import os
from struct import *
import pygame
import time
import numpy as np

from dotenv import load_dotenv
from rs2_device_manager import Rs2DeviceManagers,dummyDeviceManager

# from udpThread import udpServerThread as udpServer
from tcpThread import tcpServerThread as tcpServer

import argparse
import socket

checks()

#%%
_checkcode = 20231026
__version__ = (0,0,3)
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

send_data_type = os.getenv('send_data_type')

#%%
current_color_frame = None
current_depth_frame = None

#%%커멘드라인 인자 
## --mode [server|test] , --camera [camera_index]

parser = argparse.ArgumentParser(description='bodyTracker')
parser.add_argument('--mode', type=str, default='test', help='server or test')
parser.add_argument('--camera', type=str, default=os.getenv('camera'), help='camera serial number')
parser.add_argument('--port', type=int, default=int(os.getenv('port')), help='port')
parser.add_argument('--debug_port', type=int, default=0, help='debug port')

#카메라가 -1 이아니라면 인자로 받은 카메라 인덱스로 설정
# if parser.parse_args().camera >= 0:
#     camera_Serial = parser.parse_args().camera

__ApplicationRunMode = parser.parse_args().mode

camera_Serial = parser.parse_args().camera
port = parser.parse_args().port
debug_port = parser.parse_args().debug_port

#%%
# print(f'model loaded success : {mask_model}')
# print('model names : ',mask_model.names)

#warmup
warm_up_img = np.zeros((screen_width, screen_height, 3), dtype=np.uint8)  # Creating a blank image with dimensions 416x416


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
remote_target = None

def makeMaskPacket(mask_results,frame,_depth_frame,camera_index):
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
            
            for idx, box in enumerate(boxes): 
                #box               
                _packet += pack('<ffff', *box)
                #center point
                x1,y1,x2,y2 = map(int,box*[_width,_height,_width,_height])

                center_x = int((x1+x2)/2)
                center_y = int((y1+y2)/2)
                #get depth
                depth = _depth_frame.get_distance(center_x,center_y)
                _packet += pack('<fff',center_x,center_y,depth)

                #write segmentation image
                cropped_image = masked_image[y1:y2, x1:x2]
                 # 이미지를 PNG로 압축합니다.
                _, png_bytes = cv.imencode('.png', cropped_image)                

                # 패킷에 이미지 크기와 이미지 데이터를 추가합니다.
                _packet += pack('<L', len(png_bytes))  # 이미지 크기
                _packet += png_bytes.tobytes()  # 이미지 데이터

            _packet = _header_packet + pack('<L',len(_packet)) + _packet


    return _packet
    
def makePosePacket(pose_results,frame,_depth_frame,camera_index):
    _packet = None    
    for result in pose_results:
        keypoints = result.keypoints.xyn.cpu().numpy()
        
        detectionCount = len(keypoints)
        _header_packet = pack('<LBBBB', _checkcode, 0x17,0,detectionCount,camera_index)
        
        _packet = b''
        for idx,np_keypoints in enumerate(keypoints):
            _packet += pack('<b', np_keypoints.shape[0] ) #keypoint count)            
            # print(np_keypoints[9])            
            for kpt in np_keypoints:
                _packet += pack('<ff', *kpt)
    
    return _header_packet + pack('<L',len(_packet)) + _packet

def makeTestPacket():    
    # lastTick을 함수의 속성으로 초기화합니다.
    # hasattr 함수는 makeTestPacket에 lastTick 속성이 있는지 확인합니다.
    if not hasattr(makeTestPacket, "lastTick"):
        makeTestPacket.lastTick = time.time()

    tick = time.time()
    deltaTick = tick - makeTestPacket.lastTick
    makeTestPacket.lastTick = tick

    _packet = pack('<LBBBB', _checkcode, 0x19, 0, 0, 0)
    
    x = np.sin(tick * 2)
    _packet += pack('<f', x)
    _packet += pack('<f', deltaTick)

    return _packet
     
def detect_proc(_depth_frame,_color_frame):

    frame = _color_frame 
    # 추론
    mask_results = mask_model(frame, conf=mask_conf_thres,device=pose_device,verbose=False,classes=[0],iou = iou_threshold) 
    pose_results = pose_model(frame, conf=mask_conf_thres,device=pose_device,verbose=False)

    mask_packet = makeMaskPacket(mask_results,frame,_depth_frame,0)
    pose_packet = makePosePacket(pose_results,frame,_depth_frame,0)
    
    return mask_packet,pose_packet
def detect_proc_pose(_depth_frame,_color_frame):

    frame = _color_frame 
    # 추론
    # mask_results = mask_model(frame, conf=mask_conf_thres,device=pose_device,verbose=False,classes=[0],iou = iou_threshold) 
    pose_results = pose_model(frame, conf=mask_conf_thres,device=pose_device,verbose=False)

    # mask_packet = makeMaskPacket(mask_results,frame,_depth_frame,0)
    pose_packet = makePosePacket(pose_results,frame,_depth_frame,0)
    
    return pose_packet

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
            # conn.send(_packet)
        elif cmd == 0x14: # req set remote target
            remote_target = conn
        elif cmd == 0x15: # req set remote target
            remote_target = None
            
        if _packet is not None:
            #send tcp
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
    print('###ok')
    

    
    print(f'send_data_type : {send_data_type}')
    send_process_msg(f'send_data_type : {send_data_type}')
    send_process_msg('###ok')
    
    try :        
        while True:
            
            #camera frame
            _depth_frame,_color_frame = depth_cam.get_frames()
            frame = np.asanyarray(_color_frame.get_data())

            if remote_target is not None:
                try:
                    if send_data_type == 'mask':
                        _mask_packet,_pose_packet = detect_proc(_depth_frame, frame)
                        if _mask_packet is not None:
                            remote_target.sendall(_mask_packet)
                    elif send_data_type == 'pose':
                        _pose_packet = detect_proc_pose(_depth_frame, frame)
                        _packet = makeTestPacket()
                        if _pose_packet is not None:
                            remote_target.sendall(_pose_packet)
                            # remote_target.sendall(_packet)
                    elif send_data_type == 'all':
                        _mask_packet,_pose_packet = detect_proc(_depth_frame, frame)
                        _packet = makeTestPacket()
                        if _mask_packet is not None and _pose_packet is not None:
                            remote_target.sendall(_mask_packet + _pose_packet + _packet)
                    elif send_data_type == 'test':
                        _packet = makeTestPacket()
                        if _packet is not None:
                            remote_target.sendall(_packet)

                    # _mask_packet,_pose_packet = detect_proc(_depth_frame, frame)
                    # if _mask_packet is not None:                        
                    #     #remote_target_mask.sendall(_mask_packet + _pose_packet)
                    #     remote_target_mask.sendall(_pose_packet)
                        
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

    # _udpServer.terminate()
    _tcpServer.terminate()

    print('server task end')    
    
def rendering_task(depth_cam):
    # pygame 초기화
    pygame.init()
    
    # 색상 설정
    black = (0, 0, 0)
    white = (255, 255, 255)
    green = (0, 255, 0)
    red = (255, 0, 0)

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

        #camera frame
        _depth_frame,_color_frame = depth_cam.get_frames()
        frame = np.asanyarray(_color_frame.get_data())

        _packet = None
        _posePacket = None

        if send_data_type == 'pose':
            _posePacket = detect_proc_pose(_depth_frame, frame)
        elif send_data_type == 'all':
            _packet,_posePacket = detect_proc(_depth_frame, frame)

        #draw all
        # Convert the frame from BGR to RGB (as OpenCV uses BGR and Pygame uses RGB)
        # Convert the NumPy array to a Pygame surface
        frame_surface = pygame.surfarray.make_surface(cv.flip(np.rot90(cv.cvtColor(frame, cv.COLOR_BGR2RGB)),0))        
        # Clear the screen by camera frame
        screen.blit(pygame.transform.scale(frame_surface, (screen_width, screen_height)), (0, 0))

        #clear black
        # screen.fill(black)

        #draw detection
        #unpack
        info_text = ''
        packet_size = 0
        if _packet is not None:

            #segmentation
            checkcode,cmd,errcode,detectionCount,_ = unpack("<LBBBB",_packet[:8])   
            if cmd != 0x16 :
                print(f'segmentation cmd error : {cmd}')
                continue

            packet_size = len(_packet)
            _check_size = unpack("<L",_packet[8:12])[0]
            _check_size += 12

            if packet_size != _check_size :
                print(f'packet size error : {packet_size} != {_check_size}')
                continue

            offset = 12
            for idx in range(detectionCount):
                x1,y1,x2,y2,cx,cy,depth = unpack("<fffffff",_packet[offset:offset+28])

                # screen_width , screen_height 곱하기
                x1 = int(x1 * screen_width)
                y1 = int(y1 * screen_height)
                x2 = int(x2 * screen_width)
                y2 = int(y2 * screen_height)
                

                #get masked image size
                offset += 28
                img_size, = unpack("<L",_packet[offset:offset+4])
                offset += 4
                #get masked image
                img = _packet[offset:offset+img_size]
                offset += img_size

                #decode and draw masked image
                nparr = np.frombuffer(img, np.uint8)
                decoded_img = cv.imdecode(nparr, cv.IMREAD_UNCHANGED)  # 이미지 디코드
                image_bytes = decoded_img.tobytes()
                image_surface = pygame.image.fromstring(image_bytes, decoded_img.shape[:2][::-1], "BGRA")
                screen.blit(image_surface, (x1, y1))

                #draw box
                pygame.draw.rect(screen, green, pygame.Rect(x1,y1,x2-x1,y2-y1), 1)    

                #put depth text
                depth_text = f"{depth:.2f}"
                depth_surface = msg_font.render(depth_text, True, green)
                screen.blit(depth_surface, (x1, y1))
            info_text += f"segmentation detectionCount : {detectionCount}"

        #pose    
        if _posePacket is not None:        
            checkcode,cmd,errcode,detectionCount,_ = unpack("<LBBBB",_posePacket[:8]) 
            packet_size = unpack("<L",_posePacket[8:12])[0]
            # print(f'detectionCount : {detectionCount}')
            if cmd != 0x17 :
                print(f'pose cmd error : {cmd}')
                continue           
            offset = 12
            for idx in range(detectionCount):
                keypointCount, = unpack("<b",_posePacket[offset:offset+1])
                offset += 1
                # 배열 로 만들기
                _keypoints = []
                for _ in range(keypointCount):
                    x,y, = unpack("<ff",_posePacket[offset:offset+8])
                    offset += 8
                    _keypoints.append([x,y])

                if keypointCount == 17:
                    # pygame.draw.circle(screen, red, (int(x * screen_width), int(y * screen_height)), 2, 0)
                    right_hand = _keypoints[9]
                    left_hand = _keypoints[10]
                    head = _keypoints[0]
                    right_shoulder = _keypoints[2]
                    left_shoulder = _keypoints[5]

                    pygame.draw.circle(screen, green, (int( head[0] * screen_width), int(head[1] * screen_height)), 10, 0)

                    pygame.draw.circle(screen, red, (int( right_hand[0] * screen_width), int(right_hand[1] * screen_height)), 5, 0)
                    pygame.draw.rect(screen, red, pygame.Rect(int( right_shoulder[0] * screen_width), int(right_shoulder[1] * screen_height), 10, 10), 1)
                else :
                    print(f'keypointCount error : {keypointCount}')
                    continue
            info_text += f"pose detectionCount : {detectionCount}"



        
        # Calculate FPS
        current_time = time.time()
        elapsed_time = current_time - last_time
        fps = 1 / elapsed_time if elapsed_time > 0 else 0
        last_time = current_time
        
        # Render FPS
        fps_text = f"FPS: {fps:.2f} , elapsed_time : {elapsed_time:.2f}"
        fps_surface = msg_font.render(fps_text, True, green)
        screen.blit(fps_surface, (10, 25))  # Adjust the position (10, 10) as needed
        
        info_surface = msg_font.render(info_text, True, green)
        screen.blit(info_surface, (10, 50))  # Adjust the position (10, 10) as needed
        
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
#print(f'opencv version {cv.__version__}')
#checks()

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
        _img_file = camera_Serial[6:]
        depth_cam = dummyDeviceManager(imagefile=_img_file)
        depth_cam.start()
        time.sleep(1)
        send_process_msg('dummy camera ok')
        
    else :            
        send_process_msg('init camera')
        print('\n##Init Realsense2 camera...\n\n')
        dm = Rs2DeviceManagers()        
        depth_cam = dm.get_device_managerBySerialNumber(camera_Serial)
        if depth_cam is None:
            print(f'##camera not found : {camera_Serial}')
            send_process_msg(f'##camera not found : {camera_Serial}')
            exit()
        print(f'##camera found : {camera_Serial}')
        depth_cam.start()

        send_process_msg('camera ok')

    send_process_msg('start process')
    if __ApplicationRunMode == 'server':        
        server_task(depth_cam)
    else :        
        rendering_task(depth_cam)

    depth_cam.stop()
    print('###Done.')
    send_process_msg('end process')
except Exception as e:
    print(e)        
    print('###Exit.')  
    send_process_msg('exit process' + str(e))
    if process_msg_socket is not None:
        process_msg_socket.close()
        