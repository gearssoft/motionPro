import tkinter as tk
# from subprocess import Popen, PIPE, STDOUT,CREATE_NEW_CONSOLE
from subprocess import Popen, PIPE, STARTUPINFO, STARTF_USESHOWWINDOW, CREATE_NO_WINDOW
import shlex
import configparser

import threading
import queue

import os
import socket

import time

import subprocess

from tkinter import filedialog

__version__ = "1.0.1"
    
# 프로세스를 관리할 클래스를 정의합니다.
class ProcessManager:
    def __init__(self, tk_root,cam_index,onOk=None,onCancel=None):
        self.tk_root = tk_root
        self.cam_index = cam_index
        self.process = None
        self.debug_port = 25500 + self.cam_index
        # self.queue = queue.Queue()

        self.config_file = 'settings.ini'
        
        _frame1 = tk.Frame(root)
        _frame1.pack(side=tk.TOP, fill=tk.X)
        # 사용자 입력을 위한 레이블과 텍스트 입력 필드 생성
        self.camera_label = tk.Label(_frame1, text="Camera Serial:")
        self.camera_label.pack(side=tk.LEFT, fill=tk.X, expand=False)
        
        self.camera_entry = tk.Entry(_frame1)
        self.camera_entry.pack(side=tk.LEFT, fill=tk.X, expand=False)

        self.port_label = tk.Label(_frame1, text="Port:")
        self.port_label.pack(side=tk.LEFT, fill=tk.X, expand=False)

        self.port_entry = tk.Entry(_frame1)
        self.port_entry.pack(side=tk.LEFT, fill=tk.X, expand=False)
        
        self.start_button = tk.Button(_frame1, text=f"Start camera {cam_index}", command= lambda: self.start_process(self_button=self.start_button))
        self.start_button.pack(side=tk.LEFT, fill=tk.X, expand=False)
        
        _frame1 = tk.Frame(root)
        _frame1.pack(side=tk.TOP, fill=tk.X)

        # 첫 번째 텍스트 위젯을 _frame2 프레임의 첫 번째 열에 배치
        self.output_text = tk.Text(_frame1, height=10)
        self.output_text.grid(row=0, column=0, sticky="nsew")
        
        self.load_settings()

        def receive_udp_data():
            # UDP 소켓 생성 및 바인딩
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            server_address = ('localhost',self.debug_port)
            sock.bind(server_address)

            while True:
                data, address = sock.recvfrom(4096)
                print('received {} bytes from {}'.format(len(data), address))
                print(data.decode())
                self.output_text.insert(tk.END, '\n')
                self.output_text.insert(tk.END, data.decode())
                if 'exit process' in data.decode():
                    self.start_button.config(state=tk.NORMAL)                    
                    break
                elif '###ok' in data.decode():
                    self.start_button.config(state=tk.NORMAL)
                    self.start_button.config(text=f"Stop camera {self.cam_index}", command= lambda: self.stop_process(self_button =self.start_button))
                    if onOk:
                        onOk()
                        

        # 스레드 시작
        thread = threading.Thread(target=receive_udp_data)
        thread.daemon = True
        thread.start()
        
    def load_settings(self):
        """설정 파일에서 카메라와 포트 값을 불러옵니다."""
        self.config = configparser.ConfigParser()
        self.config.read(self.config_file)
        # 'Settings' 섹션에서 'camera'와 'port'를 가져옵니다. 없으면 기본값을 사용합니다.
        self.camera = self.config.get(f"camera_{self.cam_index}", 'serial', fallback='')
        self.port = self.config.get(f'camera_{self.cam_index}', 'port', fallback='')        

        # GUI 요소에 값을 설정합니다.
        self.camera_entry.insert(0, self.camera)
        self.port_entry.insert(0, self.port)

    def save_settings(self):
        """현재 카메라와 포트 값을 설정 파일에 저장합니다."""
        self.config[f'camera_{self.cam_index}'] = {
            'serial': self.camera_entry.get(),
            'port': self.port_entry.get()
        }        
        with open(self.config_file, 'w') as configfile:
            self.config.write(configfile)

    # 프로세스 시작 메소드
    def start_process(self, self_button):
        self.output_text.delete('1.0', tk.END)  # 텍스트 위젯의 내용을 지움
        self.output_text.insert(tk.END, "Process starting...please wait \n")  # 텍스트 위젯에 출력을 추가
        # deactivate button
        self_button.config(state=tk.DISABLED)
        camera = self.camera_entry.get()
        port = self.port_entry.get()
        # 커맨드라인 명령어를 안전하게 구성
        command = f"python bodyMasker.py --camera={camera} --port={port} --mode=server --debug_port={self.debug_port}"
        self.save_settings()
        
        # subprocess를 이용해 프로세스 시작
        args = shlex.split(command)
        
        # 윈도우에서 콘솔 창이 생성되지 않도록 설정
        startupinfo = STARTUPINFO()
        startupinfo.dwFlags |= STARTF_USESHOWWINDOW
        startupinfo.wShowWindow = 0  # 0은 SW_HIDE와 동일합니다.
        
        self.process = Popen(args, stdout=PIPE, stderr=PIPE, bufsize=1, text=True,
                            startupinfo=startupinfo, creationflags=CREATE_NO_WINDOW)

    # 프로세스 중지 메소드
    def stop_process(self,self_button):
        if self.process:
            self.process.terminate()  # 프로세스에 종료 신호를 보냅니다.
            self.process = None
            self_button.config(text=f"Start camera {self.cam_index}", command= lambda: self.start_process(self_button =self.start_button))
            # 지우기            
            self.output_text.insert(tk.END, "\nProcess stopped.\n")

if __name__ == "__main__":
# 메인 윈도우 생성 및 앱 실행
    root = tk.Tk()
    root.geometry("800x600")
    root.title("Body Masker v1.0 rev.2")
    root.iconbitmap("icon.ico")

    config = configparser.ConfigParser()
    # settings.ini 파일 읽기
    config.read('settings.ini')

    # 'system' 섹션의 'auto_start' 키 값을 불리언 값으로 읽으며, 키가 없거나 값이 명확하지 않을 경우 False를 반환
    auto_start = config.getboolean('system', 'auto_start', fallback=False)
    app_path = config.get('system', 'app_path', fallback='')

    print(f'auto_start: {auto_start}')
    
    
    bNextStep = False
    
    def onOkStep1():
        global bNextStep
        bNextStep = True
        print('onOkStep1')
    def onOkStep2():
        global bNextStep
        bNextStep = True
        print('onOkStep2')

    proMngUI1 = ProcessManager(tk_root=root,cam_index=1, onOk=onOkStep1)
    proMngUI2 = ProcessManager(tk_root=root,cam_index=2, onOk=onOkStep2)
    
    def choose_file():
        # Set the options for the file dialog
        filetypes = (
            ('executable files', '*.exe'),
            ('All files', '*.*')
        )
        filename = filedialog.askopenfilename(title='Open a file', initialdir='/', filetypes=filetypes)
        
        # Update the entry with the selected file path
        _app_path_entry.delete(0, tk.END)  # Clear the existing entry content
        _app_path_entry.insert(0, filename)  # Insert the chosen file path
        
    # auto start check box
    _frame1 = tk.Frame(root)
    _frame1.pack(side=tk.TOP, fill=tk.X)
    auto_start_var = tk.BooleanVar(value=auto_start)
    _auto_start_check = tk.Checkbutton(_frame1, text="Auto start", variable=auto_start_var)
    _auto_start_check.pack(side=tk.LEFT, fill=tk.X, expand=False)
    
    
    # application path
    _frame1 = tk.Frame(root)
    _frame1.pack(side=tk.TOP, fill=tk.X)
        # 사용자 입력을 위한 레이블과 텍스트 입력 필드 생성
    _label = tk.Label(_frame1, text="App Path:")
    _label.pack(side=tk.LEFT, fill=tk.X, expand=False)
    _app_path_entry = tk.Entry(_frame1)
    _app_path_entry.insert(0, app_path)
    _app_path_entry.pack(side=tk.LEFT, fill=tk.X, expand=False)
    
    # Button to open the file dialog
    _browse_button = tk.Button(_frame1, text="Browse...", command=choose_file)
    _browse_button.pack(side=tk.LEFT, fill=tk.X, expand=False)
    
    # save ini file button
    def _saveIniFile():
        proMngUI1.save_settings()
        proMngUI2.save_settings()
        config['system'] = {
            'auto_start': 'True',
            'app_path': _app_path_entry.get()
            }
        
        with open('settings.ini', 'w') as configfile:
            config.write(configfile)
            
    
    _frame1 = tk.Frame(root)        
    _frame1.pack(side=tk.TOP, fill=tk.X)
    _save_button = tk.Button(_frame1, text="Save", command= _saveIniFile)
    _save_button.pack(side=tk.LEFT, fill=tk.X, expand=False)
    

    # 아래 코드는 별도의 쓰레드로 처리해야 합니다.
    def _auto_start():            
        global bNextStep
        time.sleep(1)
        proMngUI1.start_process(proMngUI1.start_button)
        
        while not bNextStep:
            time.sleep(1)
            
        bNextStep = False
        time.sleep(1)
        proMngUI2.start_process(proMngUI2.start_button)
        
        while not bNextStep:
            time.sleep(1)
            
        # execute app 
        app_path = _app_path_entry.get()
        if app_path:
            print(f'execute app: {app_path}')
            subprocess.run(f'"{app_path}"', check=True)
        else:
            print('app path is empty')
            
    if auto_start:    
        thread = threading.Thread(target=_auto_start)
        thread.daemon = True
        thread.start()


    root.mainloop()
