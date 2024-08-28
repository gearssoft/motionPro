# setup

## 필수 패키지 설치

```bash
pip install -r requirements.txt
```
window에서는 다음과 같이 설치  
msvc-runtime 도 설치해야 함  



# 바디마스커 

## 실행방법

--camera : 카메라 디바이스 일련번호  
--port : 포트번호  
--mode : 실행모드 (server, test)  

```bash
python bodyMasker.py --camera=241122301397 --port=22261 --mode=server
python bodyMasker.py --camera=241122303432 --port=22262 --mode=server
python bodyMasker.py --camera=image_akb48_1.jpg --port=22262 --mode=server
python bodyMasker.py --camera=image_akb48_1.jpg --port=22262 --mode=test
```


## 실행 파일 만들기

```bash
pyinstaller --onefile --clean -w -i="icon.ico" bodyMaskerUI.py
pyinstaller --onefile --clean -w -i="icon.ico" --add-data "icon.ico;." bodyMaskerUI.py

```

## 데이터 프로세스

bodyMasker는 신체의 마스크된이미지와 포즈분석을위한 신체 각부위 키포인트 데이터를 추론과 동시에 접속된 포트로 전송함.  
여러대의 카메라를 구분하여 처리하기위하여 카메라당 tcp포트를 각각 열어서 구분함  
mrScreenTcp.cs 를 카메라마다 하나씩 배당하여 처리한다  
좌표계는 카메라 이미지크기 비율에 따른 (0~1.0) 사이의 값을 가지는 좌표계  

## 패킷 구조

**마스크 패킷(0x16)**  
x1,y1,x2,y2 : f32 X 4  
center_x,center_y,depth : f32 X 3    
imageBufferLength :  f32  
imgbuffer :  n-bytes(png format)   

**키포인트 패킷(0x17)**  
keypoint_count : byte  
keypoint_data : f32 X 2 X keypoint_count  



