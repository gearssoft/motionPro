import socket
import threading
import time

class tcpServerThread(threading.Thread):
    def __init__(self, port, onPacket,onClose,onConnect):
        threading.Thread.__init__(self)
        
        self.port = port
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.bind(('', port))
        self.tcp_socket.listen(1)  # Listen for incoming connections, 1 is the backlog
        
        self.tcp_socket.settimeout(1)  # 1초 타임아웃 설정

        self.onPacket = onPacket
        self.onClose = onClose
        self.onConnect = onConnect

        self.termination_requested = False
        
        print(f'tcp server start port = {port}')
        
    def run(self):
        while not self.termination_requested:
            try:
                if self.tcp_socket == None:
                    break
                else:
                    conn, addr = self.tcp_socket.accept()  # Block until a client connects
                    self.handle_client(conn, addr)
            except socket.timeout:
                continue
            except Exception as ex:
                print(f'error : {ex}')
                time.sleep(1)
                
        print('tcp server thread terminated')
                
        self.tcp_socket.close()
        self.tcp_socket = None
    
    def handle_client(self, conn, rinfo):
        print(f'Connected by {rinfo}')

        self.onConnect(conn, rinfo)

        while not self.termination_requested:
            try:
                _data = conn.recv(65535)  # Block until data is received
                if not _data:
                    break  # Connection closed
                self.onPacket(conn,_data, rinfo)
                # Optionally send data back to the client
                # conn.sendall(response_data)
                
            except socket.timeout:
                continue
            except Exception as ex:
                print(f'error : {ex}')
                break  # Break on any other exception
        self.onClose(conn, rinfo)
        conn.close()        
        

    def terminate(self):
        self.termination_requested = True

    def isRunning(self):
        return not self.termination_requested
