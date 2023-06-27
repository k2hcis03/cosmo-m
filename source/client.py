#! /usr/bin/python3

import socket
import json
import time
import threading
import smbus
import numpy as np

from threading import Timer
from multiprocessing import shared_memory
from konfig import Config
import configparser

class UnitBoardGetStatus(threading.Thread):
    def __init__(self, logging, event, tcp_queue, client, max_unit_board, shared_memory):
        threading.Thread.__init__(self)
        self.daemon = True
        self.logging = logging
        self.event = event
        self.tcp_queue = tcp_queue
        self.client = client
        self.max_unit_board = max_unit_board
        self.shared_memory = shared_memory
        self.send_index = 0
        self.config_file = configparser.ConfigParser()  ## 클래스 객체 생성
        self.config_file.read('/home/pi/Projects/cosmo-m/config/config.ini')  ## 파일 읽기
    
        self.send_data = []
        self.make_json_data()
        
        logging.info(f'status read thread  is running') 
        
    def make_json_data(self):
        common_config = self.config_file['common']
        size = int(common_config['SHARED_MEMORY_SIZE'])
  
        if self.send_index >= 1000000:
            self.send_index = 0
            
        self.send_data = {
            "INDEX":f"{self.send_index}",
            "DATE":f"{time.strftime('%Y-%m-%d', time.localtime(time.time()))}",
            "TIME":f"{time.strftime('%H:%M:%S')}",
            "VALUES":[],
            "STATUS":[],
            "CODE":{"CODE":1000,"MSG":"OK"}
        }
        temp_index = [16, 17, 12, 14]           #온도 값이 저장되는 shared_memory 위치
        humi_index = [13, 15]                   #습도 값이 저장되는 shared_memory 위치
        co2_index = [13, 15]                    #Co2 값이 저장되는 shared_memory 위치
        self.send_index += 1
                            
        for i in range(int(common_config['FERMEN_TANK'])):
            unit_config = self.config_file[f'unit_board{i+1}']
            for x in range(int(unit_config['TEMP_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":100+i,"SENSOR_ID":100+x,"VALUE":f"{self.shared_memory[i*size+temp_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['HUMI_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":100+i,"SENSOR_ID":200+x,"VALUE":f"{self.shared_memory[i*size+humi_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['CO2_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":100+i,"SENSOR_ID":300+x,"VALUE":f"{self.shared_memory[i*size+co2_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['LOAD_CELL'])):
                self.send_data['VALUES'].append({"TANK_ID":100+i,"SENSOR_ID":400+x,"VALUE":f"{self.shared_memory[i*size+11]*0.01:0.2F}"})
            for x in range(int(unit_config['VALVE_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":100+i,"SENSOR_ID":500+x,"VALUE":
                    f"{(self.shared_memory[i*size+6] & (0x000000FF << x*8)) >> x*8}"})
            for x in range(int(unit_config['MOTOR_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":100+i,"SENSOR_ID":600+x,"VALUE":f"{self.shared_memory[i*size+10]}"})
            
            self.send_data['STATUS'].append({"TANK_ID":100+i,"STAGE":200,"STATUS":"None"}) 
            
        cnt = int(common_config['FERMEN_TANK'])
        
        for i in range(int(common_config['BLEND_TANK'])):
            unit_config = self.config_file[f'unit_board{cnt+1}']
            for x in range(int(unit_config['TEMP_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":200+i,"SENSOR_ID":100+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+temp_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['HUMI_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":200+i,"SENSOR_ID":200+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+humi_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['CO2_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":200+i,"SENSOR_ID":300+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+co2_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['LOAD_CELL'])):
                self.send_data['VALUES'].append({"TANK_ID":200+i,"SENSOR_ID":400+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+11]*0.01:0.2F}"})
            for x in range(int(unit_config['VALVE_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":200+i,"SENSOR_ID":500+x,"VALUE":
                    f"{(self.shared_memory[(i+cnt)*size+6] & (0x000000FF << x*8)) >> x*8}"})
            for x in range(int(unit_config['MOTOR_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":200+i,"SENSOR_ID":600+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+10]}"})

            self.send_data['STATUS'].append({"TANK_ID":200+i,"STAGE":200,"STATUS":"None"}) 
            
        cnt = int(common_config['FERMEN_TANK']) + int(common_config['BLEND_TANK'])  
        for i in range(int(common_config['PROD_TANK'])):
            unit_config = self.config_file[f'unit_board{cnt+1}']
            for x in range(int(unit_config['TEMP_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":100+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+temp_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['HUMI_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":200+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+humi_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['CO2_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":300+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+co2_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['LOAD_CELL'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":400+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+11]*0.01:0.2F}"})
            for x in range(int(unit_config['VALVE_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":500+x,"VALUE":
                    f"{(self.shared_memory[(i+cnt)*size+6] & (0x000000FF << x*8)) >> x*8}"})
            for x in range(int(unit_config['MOTOR_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":600+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+10]}"})   
            
            self.send_data['STATUS'].append({"TANK_ID":400+i,"STAGE":200,"STATUS":"None"}) 
            
        cnt = int(common_config['FERMEN_TANK']) + int(common_config['BLEND_TANK']) + int(common_config['PROD_TANK'])   
        for i in range(int(common_config['CHILER_TANK'])):
            unit_config = self.config_file[f'unit_board{cnt+1}']
            for x in range(int(unit_config['TEMP_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":100+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+temp_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['HUMI_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":200+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+humi_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['CO2_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":300+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+co2_index[x]]*0.01:0.2F}"})
            for x in range(int(unit_config['LOAD_CELL'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":400+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+11]*0.01:0.2F}"})
            for x in range(int(unit_config['VALVE_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":500+x,"VALUE":
                    f"{(self.shared_memory[(i+cnt)*size+6] & (0x000000FF << x*8)) >> x*8}"})
            for x in range(int(unit_config['MOTOR_NUM'])):
                self.send_data['VALUES'].append({"TANK_ID":400+i,"SENSOR_ID":600+x,"VALUE":f"{self.shared_memory[(i+cnt)*size+10]}"})   
            
            self.send_data['STATUS'].append({"TANK_ID":400+i,"STAGE":200,"STATUS":"None"}) 
        
    def run(self):
        while True:
            try:  
                if self.event.is_set():
                    self.event.clear()
                    break
                for x in range(self.max_unit_board):            # 유닛보드마다 1초마다 get_status명령어 수행
                    data = {"unit_id" : x + 1, "cmd":"GET_STATUS", "send" : False}
                    self.tcp_queue.put(data)
                    time.sleep(0.1)
                self.make_json_data()   
                time.sleep(0.5)                                 # shared memory에서 지연시간이 없으면 문제 발생 
                ######################################################################################################  
                # 2023-06619-@K2H 
                # 서버에 데이터를 전송하기 전에 필요 데이터 정렬
                if 0:       #not self.client._closed:
                    self.client.sendall(bytes(json.dumps(self.send_data), 'UTF-8')) 
                ######################################################################################################
                time.sleep(1)             
            except Exception as e:
                time.sleep(0.5)
                print(e)
                    
class TcpClientThread(threading.Thread):
    def __init__(self, ip, port, tcp_queue, logging, GPIOADDR, shared_object, 
                 i2c_semaphor, MAXUNITBOARD, shm_name, unit_np_shm):
        threading.Thread.__init__(self)
        self.daemon = True
        self.ip = ip
        self.port = port
        self.logging = logging
        self.tcp_queue = tcp_queue
        self.shared_object = shared_object
        self.GPIOADDR = GPIOADDR
        self.i2c_semaphor = i2c_semaphor
        self.event = threading.Event()
        self.max_unit_board = MAXUNITBOARD
        self.status_thread = None
        self.shm_name = shm_name
        self.unit_np_shm = unit_np_shm
         
        self.new_shm = shared_memory.SharedMemory(name=self.shm_name)
        self.shared_memory = np.ndarray(unit_np_shm.shape, dtype=unit_np_shm.dtype, buffer=self.new_shm.buf)
    
    def run(self):
            self.logging.info('클라이언트 동작')
            SERVER_ADDR = (self.ip, self.port)
            while True:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
                        client.connect(SERVER_ADDR)
                        self.logging.info(f'서버에 연결 되었습니다.{client}')
                        
                        if self.status_thread and self.status_thread.is_alive():
                            self.event.set()        #기존 쓰레드 소멸
                            time.sleep(5)
                        
                        self.i2c_semaphor.acquire()
                        i2cbus = smbus.SMBus(1) 
                        i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF)
                        i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF)
                        i2cbus.close()
                        self.i2c_semaphor.release()
                        self.shared_object.insert(0, client)        #공유 소켓
                        self.client_socket = client
                        
                        # 상태 리드 관련 쓰레드 생성 ##################################################
                        if self.status_thread:
                            del self.status_thread
                        self.status_thread = UnitBoardGetStatus(self.logging, self.event, self.tcp_queue, self.client_socket, 
                                                                self.max_unit_board, self.shared_memory)
                        
                        self.status_thread.start()            #테스트용으로 주석 처리하고 동작 시, 주석 해제.
                        
                        while True:
                            data = json.loads(client.recv(1024).decode('UTF-8'))
                            self.tcp_queue.put(data)
                            # self.client.sendall(bytes(json.dumps({"status":"success!"}), 'UTF-8'))
                except Exception as e:
                    client.close()
                    time.sleep(0.5)
                    i2cbus = smbus.SMBus(1) 
                    i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF)
                    i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF)
                    time.sleep(0.5)
                    i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0x00)
                    i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0x00)
                    i2cbus.close()
                    print(e)
