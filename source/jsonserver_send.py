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
                NUM = int(input("\n명령어를 넣어 주세요 1=MOTOR, 2=SET GPIO, 3=GET_ADC, 4=GET_STATUS, 5 = START_TEMP 6 = STOP_TEMP: 7=TEMP_RPM: "))
                # print(NUM)
                if NUM == 1:
                    self.request.send(bytes(json.dumps({"CMD":"REF","TANK_ID":"100","STAGE":"101","STEP":"3600","DATA":[{"INDEX":"0","TEMP":"15.0","TEMP_MGN":"5.0","M_TIME":"300","M_RPM":"60"},{"INDEX":"1","TEMP":"15.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"2","TEMP":"15.3","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"3","TEMP":"15.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"4","TEMP":"16.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"5","TEMP":"16.6","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"6","TEMP":"17.0","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"7","TEMP":"17.4","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"8","TEMP":"17.9","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"9","TEMP":"18.3","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"10","TEMP":"18.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"11","TEMP":"19.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"12","TEMP":"19.6","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"13","TEMP":"20.0","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"14","TEMP":"20.4","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"15","TEMP":"21.4","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"16","TEMP":"21.7","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"17","TEMP":"21.9","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"18","TEMP":"22.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"19","TEMP":"22.3","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"20","TEMP":"22.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"21","TEMP":"22.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"22","TEMP":"22.8","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"23","TEMP":"23.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"24","TEMP":"23.4","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"25","TEMP":"23.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"26","TEMP":"24.0","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"27","TEMP":"24.3","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"28","TEMP":"24.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"29","TEMP":"25.0","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"30","TEMP":"25.3","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"31","TEMP":"25.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"32","TEMP":"24.5","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"33","TEMP":"24.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"34","TEMP":"24.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"35","TEMP":"24.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"36","TEMP":"24.6","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"37","TEMP":"24.6","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"38","TEMP":"24.6","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"39","TEMP":"24.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"40","TEMP":"24.7","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"41","TEMP":"24.8","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"42","TEMP":"24.9","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"43","TEMP":"24.9","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"44","TEMP":"24.9","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"45","TEMP":"25.0","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"46","TEMP":"25.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"47","TEMP":"25.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"48","TEMP":"25.2","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"49","TEMP":"25.2","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"50","TEMP":"25.2","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"51","TEMP":"25.2","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"52","TEMP":"25.2","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"53","TEMP":"25.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"54","TEMP":"25.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"55","TEMP":"25.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"56","TEMP":"25.1","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"57","TEMP":"25.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"58","TEMP":"25.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"59","TEMP":"25.0","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"60","TEMP":"25.0","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"61","TEMP":"24.9","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"62","TEMP":"24.8","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"63","TEMP":"24.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"64","TEMP":"24.7","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"65","TEMP":"24.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"66","TEMP":"24.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"67","TEMP":"24.6","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"68","TEMP":"24.6","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"69","TEMP":"24.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"70","TEMP":"24.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"71","TEMP":"24.3","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"72","TEMP":"24.3","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"73","TEMP":"24.3","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"74","TEMP":"24.2","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"75","TEMP":"24.2","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"76","TEMP":"24.2","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"77","TEMP":"24.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"78","TEMP":"24.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"79","TEMP":"24.0","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"80","TEMP":"23.9","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"81","TEMP":"23.9","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"82","TEMP":"23.8","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"83","TEMP":"23.7","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"84","TEMP":"23.6","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"85","TEMP":"23.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"86","TEMP":"23.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"87","TEMP":"23.5","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"88","TEMP":"23.4","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"89","TEMP":"23.4","TEMP_MGN":"5.0","M_TIME":"60","M_RPM":"240"},{"INDEX":"90","TEMP":"23.3","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"91","TEMP":"23.3","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"92","TEMP":"23.2","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"93","TEMP":"23.2","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"},{"INDEX":"94","TEMP":"23.1","TEMP_MGN":"5.0","M_TIME":"0","M_RPM":"0"}]}), 'UTF-8'))
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
                                                    "NUM" : [0, 1, 2, 3], 
                                                    "VALUE" : [False, False, False, False]}), 'UTF-8'))
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
                                                    "TIMEOUT" : 28800}), 'UTF-8'))   
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