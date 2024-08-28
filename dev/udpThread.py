import socket
import threading
import time
from struct import pack,unpack
# import json

class udpServerThread(threading.Thread):
    def __init__(self, port,onPacket):
        
        threading.Thread.__init__(self)
        
        self.port = port
        
        
        self.data = None
        self.onPacket = onPacket
        self.termination_requested = False
        
        print(f'udp server start port = {port}')

    def init_socket(self):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('', self.port))
        self.udp_socket.setblocking(0)
        self.udp_socket.settimeout(3)
        
    def run(self):

        self.init_socket()
        
        while not self.termination_requested:
            try:
                _data, _rinfo = self.udp_socket.recvfrom(65535)
                self.onPacket(self,_data,_rinfo)
                
            except socket.timeout:
                continue
            except ConnectionResetError:
                print("Connection reset by peer, reinitializing socket...")
                _packet = pack('<LBBBB',10054,0,0,0,0)
                self.onPacket(self,_packet,None)
            except Exception as ex:
                print(f'error : {ex}')
                time.sleep(1)

    def terminate(self):
        self.termination_requested = True  
    def isRunning(self):
        return not self.termination_requested        

    def send(self,data,rinfo):
        try :
            self.udp_socket.sendto(data,rinfo)
        except Exception as ex:
            print(f'error : {ex}')
        
        
class udpRecvThread(threading.Thread):
    def __init__(self, udp_socket,onPacket,recvBufferSize=65535):
        threading.Thread.__init__(self)
        
        self.udp_socket = udp_socket
        self.termination_requested = False
        self.onPacket = onPacket
        self.recvBufferSize = recvBufferSize        

    def run(self):
        
        while not self.termination_requested:
            try:
                _data, _rinfo = self.udp_socket.recvfrom(self.recvBufferSize)
                self.onPacket(_data,_rinfo)
                
            except socket.timeout:
                continue
            except Exception as ex:
                print(f'error : {ex}')
                time.sleep(1)
        print('udpRecvThread terminated')

    def terminate(self):
        self.termination_requested = True        

    