#%%
import cv2 as cv
import numpy as np

import time

from IPython.display import display
import PIL.ImageFont as ImageFont
import PIL.ImageDraw as ImageDraw
import PIL.ImageColor as ImageColor
import PIL.Image as Image

from ultralytics import YOLO,checks
from ultralytics.engine.results import Results

import time
from struct import *
import pygame

import queue
import threading

from rs2_device_manager import Rs2DeviceManagers
#%%

checks()

#%%
model = YOLO("yolov8m-seg.pt")  
#warmup
warm_up_img = np.zeros((640, 640, 3), dtype=np.uint8)  # Creating a blank image with dimensions 416x416
print('Warming up the model...')
model(warm_up_img)  # Performing a warm-up inference
print(f'Model warmed up. ok ')
#%%
dm = Rs2DeviceManagers()
_device = dm.get_device_manager(0)
_device.start()

#get resolution
cam_info = _device.getCameraInfo()[-1]

print(f"cam_info: {cam_info}")
screen_width = cam_info[0]
screen_height = cam_info[1]

print('camera start ok')
#%%


#%% 쓰레드 관련 객체 선언
# 공유 데이터 큐 생성
data_queue = queue.Queue()
cap_data_queue = queue.Queue(maxsize=1)
msg_queue = queue.Queue()

#종료 이벤트 객체 생성
shutdown_event = threading.Event()

# AI 추론 태스크 
def inference_task():
    try :
        # model = YOLO(config['WEIGHT'])  # load a pretrained YOLOv8n detection model
        model = YOLO("yolov8m-seg.pt")  
        
        # print(f'model loaded success : {config["WEIGHT"]}')
        # print('model names : ',model.names)
        # Load class names
        class_names = model.names
        
        msg_queue.put('model loaded success')
        
        #warmup
        warm_up_img = np.zeros((640, 640, 3), dtype=np.uint8)  # Creating a blank image with dimensions 416x416
        print('Warming up the model...')
        msg_queue.put('Warming up the model... please wait')
        time.sleep(0.5)
        model(warm_up_img)  # Performing a warm-up inference
        print(f'Model warmed up. ok')
        msg_queue.put('model warmed up ok')
        
        #1초 대기
        time.sleep(5)
        
        msg_queue.put('AI Model Ready')
        
        # conf_threshold = config['CONF_THRESHOLD']
        conf_threshold = 0.5
        
        while shutdown_event.is_set() == False :
            try :
                success, frame = cap_data_queue.get(block=False)
            except queue.Empty:
                # 큐가 비어 있는 경우 처리
                continue
            # print(f'cap_data_queue success : {success}')
            if success:
                results = model(source=frame, conf=conf_threshold, verbose=False)
                data_queue.put((results,class_names))  # 추론 결과를 큐에 넣기
        
    except Exception as e:
        print(e)
        print('model load failed')
        # quit()
    
    print('inference_task end')
    
    

#%%
# 화면 랜더링 태스크
def rendering_task():
    # pygame 초기화
    pygame.init()
    
    screen = pygame.display.set_mode((screen_width, screen_height))
    
    break_flag = False
    _prev_msg_text = None    
    _prev_result = None
    msg_font = pygame.font.Font(None, 24)
    
    while break_flag == False:
        start_time = time.time()  # 프레임 처리 시작 시간을 저장합니다.
        
        _depth_frame,_color_frame = _device.get_frames()
        
        frame = np.asanyarray(_color_frame.get_data())
        
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        frame_rgb = np.rot90(frame_rgb)
        
        #좌우 반전
        frame_rgb = cv.flip(frame_rgb, 0)
        
        frame_surface = pygame.surfarray.make_surface(frame_rgb)
        
        screen.blit(frame_surface, (0, 0))
        
        ## 카메라전달 
        try:
            cap_data_queue.put((True,frame), block=False)
        except queue.Full:
            # 큐가 가득 찬 경우 처리
            pass
        
        # 결과 받기 
        results = []
        try:
            results, class_names = data_queue.get(block=False)
            _prev_result = results
        except queue.Empty:
            # 큐가 비어 있는 경우 처리
            if _prev_result != None:
                results = _prev_result
            pass
        
        # 새그먼트 출력
        img_h, img_w, _ = frame.shape
        
        for result in results:
            # 각 박스에 대해 반복
            for idx, box in enumerate(result.boxes):
                # 클래스 이름이 'person'인 경우만 윤곽선을 그립니다.
                if result.names[int(box.cls)] == 'person':
                    
                    _segment = result.masks.xyn[idx]  # 이 박스에 해당하는 세그먼트를 가져옵니다. (노멀라이즈된 좌표값)
                    # 좌표값을 이미지 크기에 맞게 정수형으로 변환
                    np_cnt = (_segment * [img_w,img_h]).astype(np.int32)    
                    
                    #draw polyline green
                    # pygame.draw.polygon(screen, (0, 255, 0), np_cnt)
                    pygame.draw.lines(screen, (0, 255, 0), True, np_cnt, 2)
                    
                    # draw rectangle red using pygame
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    pygame.draw.rect(screen, (255, 0, 0), (x1, y1, x2 - x1, y2 - y1), 2)  # 마지막 인자는 선의 두께입니다.
                    
                    # display distence center
                    # x1, y1, x2, y2 = map(int, box.xyxy[0])
                    # cx = int((x1+x2)/2)
                    # cy = int((y1+y2)/2)
                    # pygame.draw.circle(screen, (255, 0, 0), (cx,cy), 5)
                    
                    # np_cnt 영역의 중점을 찾습니다.
                    M = cv.moments(np_cnt)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX, cY = 0, 0  # set center to 0 if there are no moments to avoid division by zero

                    # 중심점에서의 거리를 얻습니다.
                    depth = _depth_frame.get_distance(cX, cY)

                    # 중심점과 거리를 화면에 표시합니다.
                    font = pygame.font.Font(None, 36)  # 폰트를 설정합니다. None은 기본 폰트를 사용하고, 36은 폰트 크기입니다.
                    text_surface = font.render(f'Distance: {depth:.2f}m', True, (255, 255, 255))  # 텍스트 서피스를 생성합니다.
                    screen.blit(text_surface, (cX, cY))  # 텍스트를 화면에 그립니다.
                    
        
        pygame.display.flip() # Pygame 화면을 갱신합니다.
        
        # Pygame 이벤트 처리 (종료 등)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break_flag = True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    break_flag = True
#%%
inference_thread = threading.Thread(target=inference_task)
inference_thread.start()

#%% 메인 루프
print('start rendering_task')
rendering_task()

time.sleep(1)
# 종료 이벤트 설정 (이 호출은 쓰레드에게 종료할 것임을 알립니다.)
shutdown_event.set()
# 스레드가 종료될 때까지 대기
inference_thread.join()
print('Done.')


# frame pygame 출력
        
        # 세그먼트 예측
        # results = model.predict(frame, conf=0.5, iou=0.7, classes=None,verbose=False)
        
        # # 새그먼트 출력
        # img_h, img_w, _ = frame.shape
        
        # for result in results:
        #     # 각 박스에 대해 반복
        #     for idx, box in enumerate(result.boxes):
        #         # 클래스 이름이 'person'인 경우만 윤곽선을 그립니다.
        #         if result.names[int(box.cls)] == 'person':
                    
        #             _segment = result.masks.xyn[idx]  # 이 박스에 해당하는 세그먼트를 가져옵니다. (노멀라이즈된 좌표값)
        #             # 좌표값을 이미지 크기에 맞게 정수형으로 변환
        #             np_cnt = (_segment * [img_w,img_h]).astype(np.int32)    
                    
                    
        #             cv.polylines(frame, [np_cnt], True, (0, 255, 0), 2)
                    
        #             # 박스를 빨간색으로 그립니다.
        #             x1, y1, x2, y2 = map(int, box.xyxy[0])
        #             cv.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 빨간색으로 박스를 그립니다.
                    
                    
        # end_time = time.time()  # 프레임 처리 종료 시간을 저장합니다.
        # fps = 1.0 / (end_time - start_time)  # FPS를 계산합니다.

        # # 화면에 FPS를 출력합니다.
        # cv.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # # 화면에 출력합니다.
        # cv.imshow('Frame', frame)