#!/usr/bin/env python3

import socketserver, subprocess, sys
from threading import Thread
from pprint import pprint
import json
import time
import socket
HOST = '192.168.0.18'
PORT = 9529

class SingleTCPHandler(socketserver.BaseRequestHandler):
    "One instance per connection.  Override handle(self) to customize action."
    def handle(self):
        # self.request is the client connection
        print(f'클라이언트가 접속했습니다:{self.client_address[0]}')

        ############################################################################################
        # json 서버 테스트
        # @K2H
        # 기본 동작은 아래 while문이 동작되고 이 문은 주석처리 되어야 함.
        # while True:
        #     data = self.request.recv(4096)  # clip input at 1Kb
        #     text = data.decode('utf-8')
        #     print('\n')
        #     pprint(json.loads(text))
        ############################################################################################
        
        self.request.settimeout(5.0)
        while True:
            try:
                num = int(input("\n명령어를 넣어 주세요 1=MOTOR, 2=SET GPIO, 3=GET_ADC, 4=GET_STATUS, 5 = START_TEMP 6 = STOP_TEMP: 7=TEMP_RPM: "))
                # print(num)
                if num == 1:
                    self.request.send(bytes(json.dumps({"unit_id" : 1,
                                                    "cmd":"SET_MOTOR",
                                                    "speed" : 200, 
                                                    "dir"   : 'FW',            #FW = forward, RV = reverse
                                                    "onoff" : 'ON'}), 'UTF-8'))
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                        
                elif num == 2:
                    self.request.send(bytes(json.dumps({"unit_id" : 1,
                                                    "cmd":"SET_GPIO",
                                                    "num" : [0, 1, 2, 3], 
                                                    "value" : [False, False, False, False]}), 'UTF-8'))
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif num == 3:
                    self.request.send(bytes(json.dumps({"unit_id" : 1,
                                                    "cmd":"GET_ADC",
                                                    "num" : [0, 1, 2, 3, 4, 5]}), 'UTF-8'))     
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif num == 4:
                    self.request.send(bytes(json.dumps({"unit_id" : 1,
                                                        "cmd":"GET_STATUS",
                                                        "send" : True, 
                                                        "raw" : True}), 'UTF-8'))
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif num == 5:
                    self.request.send(bytes(json.dumps({"unit_id" : 1,
                                                    "cmd":"START_TEMP",  
                                                    "mode" : 'BOTH',
                                                    "time_out" : 28800}), 'UTF-8'))   #BOTH --> value & motor, MOTOR --> motor, 
                                                                                    #VALVE --> valve, time --> sec
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif num == 6:
                    self.request.send(bytes(json.dumps({"unit_id" : 1,
                                                    "cmd":"STOP_TEMP",  
                                                    "time_out" : 28800}), 'UTF-8'))   #BOTH --> value & motor, MOTOR --> motor, 
                                                                                    #VALVE --> valve, time --> sec
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif num == 7:
                    self.request.send(bytes(json.dumps({"unit_id" : 1,
                                                    "cmd":"TEMP_RPM",
                                                    "speed" : 100, 
                                                    "dir"   : 'FW',            #FW = forward, RV = reverse
                                                    "onoff" : 'ON', 
                                                    "time" : 3600, 
                                                    "time_out" : 28800}), 'UTF-8'))   
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                else:
                    print("잘못된 선택입니다.")
                # time.sleep(1)
            except ValueError as e:
                print(e)
        # self.request.send(bytes(json.dumps({"status":"success!"}), 'UTF-8'))
        self.request.close()

class SimpleServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    # Ctrl-C will cleanly kill all spawned threads
    daemon_threads = True
    # much faster rebinding
    allow_reuse_address = True

    def __init__(self, server_address, RequestHandlerClass):
        socketserver.TCPServer.__init__(self, server_address, RequestHandlerClass)

if __name__ == "__main__":
    server = SimpleServer((HOST, PORT), SingleTCPHandler)
    # terminate with Ctrl-C
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        sys.exit(0)