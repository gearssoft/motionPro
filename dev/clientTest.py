#%%
import socket

import sys
import time 
import cv2 as cv
import numpy as np
import datetime
import yaml
import os
from struct import *

import PIL.Image as Image
import PIL.ImageColor as ImageColor
import PIL.ImageDraw as ImageDraw
import PIL.ImageFont as ImageFont
from IPython.display import display
import matplotlib.pyplot as plt

import threading
from udpThread import udpRecvThread

import tkinter as tk
from tkinter import scrolledtext,messagebox

from dotenv import load_dotenv

#%% .env 파일 로드

load_dotenv()
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
clientSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

def append_output(text):
    output_text.insert(tk.END, text + '\n')
    output_text.see(tk.END)
def clear_output():
    output_text.delete('1.0', tk.END)
    append_output("Cleared output")

def onPack(data,rinfo):
    
    _checkcode,cmd,errcode,*param = unpack('<LBBBB',data[:8])
    _checkcode, cmd, errcode, *param = unpack('<LBBBB', data[:8])
    text = f'checkcode : {_checkcode}, cmd : {cmd}, errcode : {errcode}, from {rinfo}'
    append_output(text)
    
    print(f'checkcode : {_checkcode}, cmd : {cmd}, errcode : {errcode}, from {rinfo}')
    
    if cmd == 0x10: #ping   
        version = (errcode,param[0],param[1])     
        print(f"app version : {version} ")
        _time = unpack('<L',data[8:12])
        print(f"timestamp : {_time[0]}")
    elif cmd == 0x11: #detect
        print(len(data))
        detect_count = param[0]
        print(f"detect count : {detect_count}")
        append_output(f"detect count : {detect_count}")
        data_offset = 8  # 헤더 크기에 따라 조정
        detection_size = 116  # 감지 정보 크기에 따라 조정
        for i in range(detect_count):
            detection_data = data[data_offset:data_offset + detection_size]
            
            _id, x1, y1, x2, y2, \
            kpt_head_x, kpt_head_y, \
            kpt_lear_x, kpt_lear_y, kpt_rear_x, kpt_rear_y, \
            kpt_lhand_x, kpt_lhand_y, kpt_rhand_x, kpt_rhand_y, \
            kpt_lelbow_x, kpt_lelbow_y, kpt_relbow_x, kpt_relbow_y, \
            kpt_lshoulder_x, kpt_lshoulder_y, kpt_rshoulder_x, kpt_rshoulder_y, \
            head_depth, lhand_depth, rhand_depth, lshoulder_depth, rshoulder_depth, timestamp = \
            unpack('<L4f18f6f', detection_data)                    

            append_output(f"id : {_id}, x1 : {x1:.2}, y1 : {y1:.2}, x2 : {x2:.2}, y2 : {y2:.2}, timestamp : {timestamp}")

            data_offset += detection_size
    elif cmd == 0x12: #req image
        print(len(data))
        _id = param[0]
        append_output(f"res image id : {_id}")
        if errcode == 0:
            _data_size = unpack('<L',data[8:12])
            print(f"data size : {_data_size}")
            append_output(f"data size : {_data_size}")
            # get image data , jpg format
            _img_data = data[12:]
            #check image data size
            if len(_img_data) == _data_size[0]:
                #convert to numpy array
                _img = np.frombuffer(_img_data,dtype=np.uint8)
                #decode jpg
                _img = cv.imdecode(_img, cv.IMREAD_UNCHANGED)                

                # Split the image into RGB and Alpha channels
                b, g, r, alpha = cv.split(_img)

                # Create a green color image
                green_img = np.zeros_like(b)
                green_img[:] = 0  # Green channel
                green_img[:] = 255  # Green channel

                # Update red, blue and green channels where alpha is 0
                r[alpha == 0] = 0
                g[alpha == 0] = 255
                b[alpha == 0] = 0

                # Merge updated channels back into the image
                _img = cv.merge((b, g, r, alpha))

                cv.imshow(f"res image id : {_id}", _img)
                cv.waitKey(0)
                cv.destroyAllWindows()
            else :
                print(f"error : image data size not match")
        else :
            print(f"error : {errcode}")
        pass
    elif cmd == 0x13: #req 3dpoint
        print(len(data))
        detect_count = param[0]
        print(f"detect count : {detect_count}")
        append_output(f"detect count : {detect_count}")

        data_offset = 8  # 헤더 크기에 따라 조정
        detection_size = 64  # 감지 정보 크기에 따라 조정
        for i in range(detect_count):
            detection_data = data[data_offset:data_offset + detection_size]
            
            unpacked_data = unpack('<L3f3f3f3f3f', detection_data)
            _id = unpacked_data[0]
            head = unpacked_data[1:4]
            lhand = unpacked_data[4:7]
            rhand = unpacked_data[7:10]
            lshd = unpacked_data[10:13]
            rshd = unpacked_data[13:16]
            

            append_output(f"id : {_id} ")
            append_output(f"head : x : {head[0]:.2}, y : {head[1]:.2}, z : {head[2]:.2} ")
            append_output("-----------------------")

            data_offset += detection_size
        

    
clientSocket.settimeout(3)
_udpRecvThread = udpRecvThread(clientSocket,onPacket=onPack)
_udpRecvThread.start()

checkcode = 20231026

#%%
shutdown_event = threading.Event()
def loop_sendDetect():
    while not shutdown_event.is_set():
        _req_header = pack('<LB',checkcode,
               0x11, # start cmd
               )
        _req_header += bytearray(3) # padding
        clientSocket.sendto(_req_header, (str(target_ip),pose_data_port) )
        time.sleep(1)


# %%
def send_ping():
    _req_header = pack('<LB', checkcode, 0x10)  # ping cmd
    _req_header += bytearray(3)  # padding
    clientSocket.sendto(_req_header, (str(target_ip), pose_data_port))
    append_output("Sent ping command")

def send_detect():
    _req_header = pack('<LB', checkcode, 0x11)  # start cmd
    _req_header += bytearray(3)  # padding
    clientSocket.sendto(_req_header, (str(target_ip), pose_data_port))
    append_output("Sent detect command")    

def req_image():
    try :
        _id = int(detect_id_entry.get())    
        _req_header = pack('<LBBBB', checkcode, 0x12,_id,0,0)  # req image cmd
        clientSocket.sendto(_req_header, (str(target_ip), pose_data_port))
        append_output(f"Sent req image command for id {_id}")
    except:
        messagebox.showerror("Error", "Invalid detect id")

def req_3dpoint():
    _req_header = pack('<LB', checkcode, 0x13)  # req 3dpoint cmd
    _req_header += bytearray(3)  # padding
    clientSocket.sendto(_req_header, (str(target_ip), pose_data_port))
    append_output("Sent req 3dpoint command")

def start_loop_detect():
    shutdown_event.clear()
    threading.Thread(target=loop_sendDetect).start()
    append_output("Started loop detect")

def stop_loop_detect():
    shutdown_event.set()
    append_output("Stopped loop detect")

def clear_output_handler():
    clear_output()


def quit_app():
    global bLoop
    bLoop = False
    root.quit()

# GUI 생성
root = tk.Tk()
root.title("BodyTracker Test GUI")

# 버튼 프레임 생성
button_frame = tk.Frame(root)
button_frame.pack(side=tk.TOP, fill=tk.X)

# 버튼 생성
ping_button = tk.Button(button_frame, text="Ping", command=send_ping)
ping_button.pack(side=tk.LEFT, fill=tk.X, expand=True)

detect_button = tk.Button(button_frame, text="Detect", command=send_detect)
detect_button.pack(side=tk.LEFT, fill=tk.X, expand=True)

detect_button = tk.Button(button_frame, text="Detect 3D", command=req_3dpoint)
detect_button.pack(side=tk.LEFT, fill=tk.X, expand=True)

loop_detect_button = tk.Button(button_frame, text="Loop Detect", command=start_loop_detect)
loop_detect_button.pack(side=tk.LEFT, fill=tk.X, expand=True)

stop_loop_detect_button = tk.Button(button_frame, text="Stop Loop Detect", command=stop_loop_detect)
stop_loop_detect_button.pack(side=tk.LEFT, fill=tk.X, expand=True)

req_image_button = tk.Button(button_frame, text="Req Image", command=req_image)
req_image_button.pack(side=tk.LEFT, fill=tk.X, expand=True)

# 버튼 프레임 생성
system_button_frame = tk.Frame(root)
system_button_frame.pack(side=tk.TOP, fill=tk.X)

clear_output_button = tk.Button(system_button_frame, text="Clear Output", command=clear_output_handler)
clear_output_button.pack(side=tk.LEFT, fill=tk.X, expand=False)

quit_button = tk.Button(system_button_frame, text="Quit", command=quit_app)
quit_button.pack(side=tk.RIGHT, fill=tk.X, expand=False)

# 포트 프레임 생성
port_frame = tk.Frame(root)
port_frame.pack(side=tk.TOP, fill=tk.X)

# 포트 레이블과 입력 창 생성
port_label = tk.Label(port_frame, text="Port:")
port_label.pack(side=tk.LEFT, padx=5, pady=5)

port_var = tk.StringVar()
def update_port_var(*args):
    global pose_data_port
    new_val = port_var.get()
    try :
        new_val = int(new_val)
        pose_data_port = new_val # 포트 번호 업데이트
        print(f'update port : {pose_data_port}')
    except:
        messagebox.showerror("Error", "Invalid port number")
        port_var.set(pose_data_port)

# port_var의 변화를 추적하도록 설정
port_var.trace_add('write', update_port_var)

port_entry = tk.Entry(port_frame,textvariable=port_var, width=10)
port_entry.pack(side=tk.LEFT, fill=tk.X, expand=False)
port_entry.delete(0, tk.END)
port_entry.insert(0, pose_data_port)

tk.Label(port_frame, text="Detect Id:").pack(side=tk.LEFT, padx=5, pady=5)
detect_id_entry = tk.Entry(port_frame, width=3)
detect_id_entry.pack(side=tk.LEFT, fill=tk.X, expand=False)
detect_id_entry.delete(0, tk.END)
detect_id_entry.insert(0, 0)


# 출력 텍스트 뷰 생성
output_text = scrolledtext.ScrolledText(root, width=80, height=20)
output_text.pack(fill=tk.BOTH, expand=True)

# GUI 이벤트 루프 시작
root.mainloop()

#%%
_udpRecvThread.terminate()