import threading
import time

class CLIThread(threading.Thread):
    def __init__(self,onCallback):
        threading.Thread.__init__(self, daemon=True)
        self.onCallback = onCallback
        self.running = True
    
    def run(self):
        while self.running:
            command = input("Enter a command: ")
            # 입력된 명령어에 따라 적절한 처리를 수행
            self.onCallback(self,command)
            # if command == "quit":
            #     break
            # # else:
            #     print(f"Command '{command}' not recognized.")
            time.sleep(0.3) # context switch to other thread
        print('cli thread end')
            
    def terminate(self):
        self.running = False
    
    