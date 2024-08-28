#%%
import cv2 as cv

# import ultralytics
from ultralytics import YOLO,checks

from IPython.display import display
import PIL.ImageFont as ImageFont
import PIL.ImageDraw as ImageDraw
import PIL.ImageColor as ImageColor
import PIL.Image as Image

import pygame
import numpy as np


checks()
#%%
# Load a model
model = YOLO('yolov8n-pose.pt')  # load an official model
#%% load mp4
cap = cv.VideoCapture('bts_1.mp4')
total_framecount = int(cap.get(cv.CAP_PROP_FRAME_COUNT)) # 전체 프레임 구하기 

print(f'total frame count {total_framecount}')

frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))


# Pygame setup
pygame.init()
screen = pygame.display.set_mode(
    (frame_width, 
     frame_height))  # Note the swapped dimensions due to rotation
clock = pygame.time.Clock()

frame_index = 340
#%%
if cap.isOpened() :
    
    bLoop = True
    
    while bLoop:
        cap.set(cv.CAP_PROP_POS_FRAMES,frame_index) #프레임 선택
        ret, frame = cap.read()
        
        # Run YOLOv8 inference on the frame
        results = model(frame, conf=0.5,verbose=False)  # predict on an image
        
        # org_frame = frame.copy()
        
        # 원본 프레임 축소 및 위치 계산
        small_frame = cv.resize(frame, (frame_width // 4, frame_height // 4))
        small_frame_rgb = cv.cvtColor(small_frame, cv.COLOR_BGR2RGB)
        small_frame_surface = pygame.surfarray.make_surface(small_frame_rgb)
        small_frame_surface = pygame.transform.rotate(small_frame_surface, -90)
        small_frame_surface = pygame.transform.flip(small_frame_surface, True, False)
        
        for det in results:
            for kpts in det.keypoints.data:
                # Each keypoint is a 2D tensor
                for kpt in kpts:
                    x, y, _ = map(int, kpt)
                    cv.circle(frame, (x, y), 5, (0, 0, 255), -1)
                    
        cv.putText(frame, f"frame :{frame_index} / {total_framecount}", (10, 50), cv.FONT_HERSHEY_COMPLEX, 1.2, (255, 0, 255), 1, cv.LINE_AA) 
        
        # cv.imshow('frame',frame)
        # cv.imshow('org_frame',org_frame)
        
        # numpy 배열(RGB 프레임)을 pygame Surface로 변환하여 그리기 수행
        
         # Convert the frame from BGR to RGB (pygame format)
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        frame_rgb = np.rot90(frame_rgb)  # Rotate for correct orientation in pygame
        
        frame_rgb = np.flipud(frame_rgb)
         
         
        frame_surface = pygame.surfarray.make_surface(frame_rgb)
        
        #display
        # Display the frame
        screen.blit(frame_surface, (0, 0))
        
        
        
        # 축소된 원본 프레임 우하단에 표시
        x_pos = frame_width - small_frame_surface.get_width()
        y_pos = frame_height - small_frame_surface.get_height()
        screen.blit(small_frame_surface, (x_pos, y_pos))
        
        pygame.display.flip()  # Update the full display Surface to the screen
        
        # Check for quit
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                bLoop = False
                break;
            elif event.type == pygame.KEYDOWN:
                print(event.key)
                if event.key == pygame.K_a:
                    frame_index += 1
                    if frame_index >= total_framecount:
                        frame_index = 0  # 순환
                    cap.set(cv.CAP_PROP_POS_FRAMES, frame_index)
                elif event.key == pygame.K_s:
                    frame_index -= 1
                    if frame_index < 0:
                        frame_index = total_framecount - 1  # 순환
                    cap.set(cv.CAP_PROP_POS_FRAMES, frame_index)

#%%
   
cap.release()
pygame.quit()

    