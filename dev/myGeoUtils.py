import pygame
from sklearn.linear_model import LinearRegression
import numpy as np


def draw_detection(fontObj,screen,detection) : 
    
    screen_width, screen_height = screen.get_size()
        
    _id,(x1,y1,x2,y2),kpt_head,kpt_lear,kpt_rear,kpt_lhand,kpt_rhand,kpt_lelbow,kpt_relbow,kpt_lshoulder,kpt_rshoulder,head_depth,lhand_depth,rhand_depth,lshoulder_depth,rshoulder_depth,time_stemp = detection
    
    # screen 좌표로 변환
    r_hand_posx, r_hand_posy = map(int, (kpt_rhand[0]*screen_width, kpt_rhand[1]*screen_height))
    l_hand_posx, l_hand_posy = map(int, (kpt_lhand[0]*screen_width, kpt_lhand[1]*screen_height))
    r_shoulder_posx, r_shoulder_posy = map(int, (kpt_rshoulder[0]*screen_width, kpt_rshoulder[1]*screen_height))
    l_shoulder_posx, l_shoulder_posy = map(int, (kpt_lshoulder[0]*screen_width, kpt_lshoulder[1]*screen_height))
    head_posx, head_posy = map(int, (kpt_head[0]*screen_width, kpt_head[1]*screen_height))    
    
    pygame.draw.rect(screen, (0, 255, 0), (x1, y1, x2-x1, y2-y1), 2)
    #머리출력
    pygame.draw.circle(screen,(255,0, 0),(head_posx, head_posy), 10, 10)
    
    #id 출력
    id_surface = fontObj.render(f"Id {_id}", True, (255, 255, 255))
    text_width, text_height = id_surface.get_size()  # 텍스트 서피스의 크기를 얻습니다.
    text_pos_x = head_posx - (text_width // 2)  # 텍스트의 x 좌표를 조정하여 정중앙에 위치하도록 합니다.
    text_pos_y = head_posy - (text_height // 2)  # 텍스트의 y 좌표를 조정하여 정중앙에 위치하도록 합니다.
    screen.blit(id_surface, (text_pos_x, text_pos_y))
    
    #거리출력
    screen.blit(fontObj.render(f"{head_depth:.2f}", True, (255, 255, 0)), (head_posx, head_posy))

    # 손과 어깨 부분에 대한 색상을 달리하여 출력
    pygame.draw.circle(screen, (0, 0, 255), (l_hand_posx, l_hand_posy), 10, 10)  # 왼손은 파란색으로 표시
    #거리출력
    screen.blit(fontObj.render(f"{lhand_depth:.2f}", True, (255, 255, 0)), (l_hand_posx, l_hand_posy))
    
    pygame.draw.circle(screen, (255, 165, 0), (r_hand_posx, r_hand_posy), 10, 10)  # 오른손은 오랜지색으로 표시
    #거리출력
    screen.blit(fontObj.render(f"{rhand_depth:.2f}", True, (255, 255, 0)), (r_hand_posx, r_hand_posy))
    
    pygame.draw.circle(screen, (135, 206, 250), (l_shoulder_posx, l_shoulder_posy), 10, 10)  # 왼쪽 어깨는 하늘색으로 표시
    #거리출력
    screen.blit(fontObj.render(f"{lshoulder_depth:.2f}", True, (255, 255, 0)), (l_shoulder_posx, l_shoulder_posy))
    
    pygame.draw.circle(screen, (153, 50, 204), (r_shoulder_posx, r_shoulder_posy), 10, 10)  # 오른쪽 어깨는 자주색으로 표시
    #거리출력
    screen.blit(fontObj.render(f"{rshoulder_depth:.2f}", True, (255, 255, 0)), (r_shoulder_posx, r_shoulder_posy))

def draw_crosshair(screen, color, size,x,y):
    pygame.draw.line(screen, color, (x - size, y), (x + size, y), 1)
    pygame.draw.line(screen, color, (x, y - size), (x, y + size), 1)

def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def calculate_trajectory_info(trajectory_queue):
    # 궤적의 점들을 가져오기
    points = np.array([(pos[0], pos[1]) for pos in list(trajectory_queue.queue)])
    # 전체 길이 계산
    total_length = np.sum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))

    # 선형 회귀를 사용하여 궤적의 점들에 최적의 직선을 맞추기
    model = LinearRegression()
    model.fit(points[:, 0].reshape(-1, 1), points[:, 1])
    line_func = lambda x: model.coef_ * x + model.intercept_

    # 직선과 각 점 사이의 거리 계산
    distances = [euclidean_distance(point, (point[0], line_func(point[0])))
             for point in points]
    linearity = np.mean(distances)

    # 기울기를 사용하여 궤적이 얼마나 수평에 가까운지 결정
    horizontalness = abs(model.coef_[0])  # 기울기의 절댓값
    
    # 궤적의 시작점과 끝점 가져오기
    start_point = np.array(trajectory_queue.queue[0][:2])
    end_point = np.array(trajectory_queue.queue[-1][:2])

    return total_length, linearity, horizontalness , end_point[0] - start_point[0]