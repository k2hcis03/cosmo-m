#!/usr/bin/env python3

import socketserver, subprocess, sys
from threading import Thread
from pprint import pprint
import json
import time
import socket
HOST = '192.168.0.18'
PORT = 7001

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
                NUM = int(input("\n 1=MOTOR, 2=SET GPIO, 3=GET_ADC, 4=GET_STATUS, 5 = START_TEMP 6 = STOP_TEMP: 7=TEMP_RPM: 8=RESET GPIO "))
                # print(NUM)
                if NUM == 1:
                    self.request.send(bytes(json.dumps({"UNIT_ID" : 0, 
                                        "TANK_ID" : "100",                 
                                        "CMD":"TEMP_RPM",
                                        "SPEED" : 1000, 
                                        "DIR"   : 'FW',            #FW = forward, RV = reverse
                                        "ONOFF" : 'ON', 
                                        "TIME" : 300,
                                        "SEND" : True}), 'UTF-8'))
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                        
                elif NUM == 2:
                    self.request.send(bytes(json.dumps({"UNIT_ID" : 0,
                                                    "TANK_ID" : "100",
                                                    "CMD":"SET_GPIO",
                                                    "NUM" : [0, 1, 2, 3, 4], 
                                                    "VALUE" : [True, False, False, False, False]}), 'UTF-8'))
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif NUM == 3:
                    self.request.send(bytes(json.dumps({"UNIT_ID" : 0,
                                                    "TANK_ID" : "100",
                                                    "CMD":"GET_ADC",
                                                    "NUM" : [0, 1, 2, 3, 4, 5]}), 'UTF-8'))     
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif NUM == 4:
                    self.request.send(bytes(json.dumps({"UNIT_ID" : 0,
                                                        "TANK_ID" : "100",
                                                        "CMD":"GET_STATUS",
                                                        "SEND" : True, 
                                                        "RAW" : True}), 'UTF-8'))
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif NUM == 5:
                    self.request.send(bytes(json.dumps({"UNIT_ID" : 0,
                                                    "TANK_ID" : "100",
                                                    "CMD":"START_TEMP",  
                                                    "MODE" : 'BOTH',
                                                    "TIMEOUT" : 28800}), 'UTF-8'))   #BOTH --> VALUE & motor, MOTOR --> motor, 
                                                                                    #VALVE --> valve, time --> sec
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif NUM == 6:
                    self.request.send(bytes(json.dumps({"UNIT_ID" : 0,
                                                    "TANK_ID" : "100",
                                                    "CMD":"STOP_TEMP",  
                                                    "TIMEOUT" : 28800}), 'UTF-8'))   #BOTH --> VALUE & motor, MOTOR --> motor, 
                                                                                    #VALVE --> valve, time --> sec
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif NUM == 7:
                    self.request.send(bytes(json.dumps({"UNIT_ID" : 0,
                                                    "TANK_ID" : "100",
                                                    "CMD":"TEMP_RPM",
                                                    "SPEED" : 100, 
                                                    "DIR"   : 'FW',            #FW = forward, RV = reverse
                                                    "ONOFF" : 'ON', 
                                                    "TIME" : 3600, 
                                                    "SEND" : True,
                                                    "TIMEOUT" : 28800}), 'UTF-8'))   
                    try:
                        data = self.request.recv(1024)  # clip input at 1Kb
                        text = data.decode('utf-8')
                        print('\n')
                        pprint(json.loads(text))
                    except socket.timeout:
                        print("Time out")
                elif NUM == 8:
                    self.request.send(bytes(json.dumps({"UNIT_ID" : 0,
                                                    "TANK_ID" : "100",
                                                    "CMD":"SET_GPIO",
                                                    "NUM" : [0, 1, 2, 3, 4], 
                                                    "VALUE" : [False, False, False, False, False]}), 'UTF-8'))
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