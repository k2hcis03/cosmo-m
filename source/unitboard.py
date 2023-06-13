#! /usr/bin/python3

import smbus
import os
import can
import time
import numpy as np
import json
from multiprocessing import Process, Queue, Manager, shared_memory
import smbus 
import threading
from multiprocessing import Queue
from datetime import datetime
from threading import Timer
from pid_controller import PID_Control
import csv

# _txt = ('갱신시간(sec)', '최소', '최대', '목표치', '비례이득(kp):', '적분이득(ki):', '미분이득(kd):')
# _dt, _min, _max, _sv, _kp, _ki, _kd = (5, 0., 5., 21., 0.1, 0.5, 0.01)
_dt, _min, _max, _sv, _kp, _ki, _kd = (5, 0., 5., 21., 0.1, 0.01, 0.01)
g_file_path = "./data/ref.json"
g_temp_control_start = False
                   
class UnitBoardTempControl(threading.Thread):
    def __init__(self, id, queue, lock, event, logging, can_fd_transmitte_queue, 
                 can_fd_receive_queue, command_queue, shared_memory, unit_semaphor, config):
        threading.Thread.__init__(self)
        self.daemon = True
        self.id = id                            # id는 0부터 시작
        self.logging = logging
        self.can_fd_transmitte_queue = can_fd_transmitte_queue
        self.can_fd_receive_queue = can_fd_receive_queue
        self.lock = lock
        self.event = event
        self.pid_event = threading.Event()
        self.queue = queue
        self.command_queue = command_queue
        self.pid = PID_Control(_dt, _min, _max, _kp, _ki, _kd)
        self.pid.update(_dt, _min, _max, _kp, _ki, _kd)
        self.shared_memory = shared_memory
        self.time_to_on = 0
        self.pid_call_time = 0
        self.unit_semaphor = unit_semaphor
        
        self.current_temp1 = 0              # 상부
        self.current_temp2 = 0              # 하부
        self.config = config
        # Setup and start the timer        
        # self.timer.start()
        
        with open(g_file_path, 'r') as file:
            self.ref_data = json.load(file)
            self.ref_step = self.ref_data['step']
            self.ref_total  = self.ref_data['total']
            self.ref_temp = self.ref_data['ref_data']
            self.logging.info(f'id : {self.id + 1} UnitBoard read reference temperature data')
    
    def pid_task(self):    
        print(f'self.id {self.id+1} pid_task{self.pid_call_time} and {self.time_to_on} \
              CT {self.current_temp2:0.2F} and FT and {self.ref_temp[0]}')
        
        if self.pid_call_time:
            if self.time_to_on:
                self.time_to_on -= 1
                message = {"unit_id" : self.id + 1,                  
                            "cmd":"SET_GPIO",
                            "num" : [0, 1, 2, 3], 
                            "value" : [False, True, False, False]}      
            else:
                message = {"unit_id" : self.id + 1,
                            "cmd":"SET_GPIO",
                            "num" : [0, 1, 2, 3], 
                            "value" : [False, False, False, False]}       
            self.command_queue.put(message)
            self.pid_call_time -= 1
            Timer(1, self.pid_task).start()
        else:
            Timer(1, self.pid_task).cancel()
            self.pid_event.set()
        
        
    def get_status(self, send_host = False):       
        # 온도계산 전에 GET_ADC를 호출 함.
        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                    data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
        
        while not self.can_fd_receive_queue.empty():
            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.

        self.can_fd_transmitte_queue.put(message) 
        time.sleep(0.05)
        
        if not self.can_fd_receive_queue.empty():
            message = self.can_fd_receive_queue.get()
            self.logging.info(f'id : {self.id + 1} Received message: {message}')
            
            self.unit_semaphor.acquire()
            inclination1 = 77.5 / (float(self.config['TEMP1_77_5']) - float(self.config['TEMP1_0']))
            y_offset1 = inclination1 * float(self.config['TEMP1_0'])
                                
            inclination2 = 77.5 / (float(self.config['TEMP2_77_5']) - float(self.config['TEMP2_0']))
            y_offset2 = inclination2 * float(self.config['TEMP2_0'])
            
            self.shared_memory[0 + self.id*20] = (np.int32)((np.int32)(message.data[5] << 8) | (np.int32)(message.data[4]))
            self.shared_memory[1 + self.id*20] = (np.int32)((np.int32)(message.data[7] << 8) | (np.int32)(message.data[6]))
            
            self.current_temp1 =  inclination1 * self.shared_memory[0 + self.id*20] - y_offset1
            self.current_temp2 =  inclination2 * self.shared_memory[1 + self.id*20] - y_offset2
            
            self.unit_semaphor.release()
        else:
            self.logging.warning(f'id : {self.id + 1} unit board is not response') 
        # GET_ADC후에 GET_STATUS 수행
        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                    data=[0xF2, 0x14, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
        
        while not self.can_fd_receive_queue.empty():
            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
        
        self.can_fd_transmitte_queue.put(message) 
        time.sleep(0.05)
        
        if not self.can_fd_receive_queue.empty():
            message = self.can_fd_receive_queue.get()
            self.logging.info(f'id : {self.id + 1} Received message: {message}')
            self.unit_semaphor.acquire()
            self.shared_memory[6 + self.id*20] = (np.int32)(message.data[7] << 24 | message.data[6] << 16 
                                            | message.data[5] << 8 | message.data[4])
            self.shared_memory[7 + self.id*20] = (np.int32)(message.data[9] << 8 | message.data[8])
            self.shared_memory[8 + self.id*20] = (np.int32)(message.data[13] << 24 | message.data[12] << 16 
                                            | message.data[11] << 8 | message.data[10])
            self.shared_memory[9 + self.id*20] = (np.int32)(message.data[14])
            self.shared_memory[10 + self.id*20] = (np.int32)(message.data[15] << 8 | message.data[16]) #RPM
            self.shared_memory[11 + self.id*20] = (np.int32)(message.data[17] << 8 | message.data[18]) #load cell
            
            self.shared_memory[12 + self.id*20] = (np.int32)(message.data[19] << 8 | message.data[20]) #Ext Temp1
            self.shared_memory[13 + self.id*20] = (np.int32)(message.data[21] << 8 | message.data[22]) #Ext Humi1
            self.shared_memory[14 + self.id*20] = (np.int32)(message.data[23] << 8 | message.data[24]) #Ext Temp2
            self.shared_memory[15 + self.id*20] = (np.int32)(message.data[25] << 8 | message.data[26]) #Ext Humi2
            self.unit_semaphor.release()
        else:
            self.logging.warning(f'id : {self.id + 1} unit board is not response')        
    def run(self):
            self.logging.info(f'id : {self.id + 1} UnitBoard Temp Control Thread Run')
            while True:
                try:
                    self.event.wait()
                    self.event.clear()
                    self.pid.update(_dt, _min, _max, _kp, _ki, _kd)
                    ##############################################################################################################
                    ## 온도 관련 동작
                    ## 20230609
                    ## @K2H
                    ## 온도 테스트를 위한 데이저 저장. 
                    self.writer_csv = open(f"test{self.id+1}.csv", 'w', encoding='utf-8', newline='')
                    self.writer = csv.writer(self.writer_csv, delimiter=',')
                    self.writer.writerow(['time'] + ['ref.temp'] + ['real temp'] + ['valve on time'])   
                    ##############################################################################################################
        
                    for x in range(self.ref_total):
                        if g_temp_control_start:
                            time_start = time.time()
                            ref_temp = self.ref_temp[x]
                            
                            while (time_start + self.ref_step) > time.time():
                                if g_temp_control_start:
                                    self.get_status()                               #현재 온도데이터 읽음.
                                    # inc, desc = self.pid.calc(ref_temp, self.current_temp1)
                                    inc, desc = self.pid.calc(self.current_temp2, ref_temp)
                                    self.time_to_on = round(inc)                         #소수점 첫번째에서 반올림
                                    self.pid_call_time = 4                               #타이머 호출 회수 -1
                                    self.writer.writerow([time.time(), ref_temp, self.current_temp2, self.time_to_on])  
                                    self.timer = Timer(1, self.pid_task).start()
                                    self.pid_event.wait()                              #1초 타이머가 5번 호출되는것을 기다림
                                    self.pid_event.clear()
                                else:
                                    self.writer_csv.close() 
                                    break
                        else:
                            self.pid.update(_dt, _min, _max, _kp, _ki, _kd)
                            break
                    ##############################################################################################################
                    
                except Exception as e:
                    self.writer_csv.close()
                    print(e)

class UnitBoard:
    def __init__(self, logging, transmitte_queue, shared_object, GPIOADDR, i2c_semaphor) -> None:
        self.can_fd_transmitte_queue = transmitte_queue
        self.logging = logging
        self.socket = shared_object
        self.GPIOADDR = GPIOADDR
        self.i2c_semaphor = i2c_semaphor
        # self.pid_update()
        
    def unit_process(self, n, shm, arr, semaphor, receive_queue, command_queue, config):
        new_shm = shared_memory.SharedMemory(name=shm)
        self.shared_memory = np.ndarray(arr.shape, dtype=arr.dtype, buffer=new_shm.buf)
        self.logging.info(f'Process {os.getpid()} and {n} are created')  
        self.id = n
        self.unit_semaphor = semaphor
        self.can_fd_receive_queue = receive_queue
        self.command_queue = command_queue
        self.config = config
        self.event = threading.Event()
        self.lock = threading.Lock()
        self.queue = Queue(12)
        
        # 온도조절 관련 쓰레드 생성 ##################################################
        temp_thread = UnitBoardTempControl(self.id, self.queue, self.lock, self.event, self.logging, self.can_fd_transmitte_queue, 
                                                           self.can_fd_receive_queue, self.command_queue, 
                                                           self.shared_memory, self.unit_semaphor, self.config)
        temp_thread.start()
        
        ###########################################################################
        # 처음 부팅이 되면 환경 설정을 유닛보드로 전송 ################################
        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x16, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
        message.data[4] = self.config['MOTOR_ID'] 
        temp = self.config['SLEEP_SPEED'] 
        message.data[6] = temp & 0xff               #big endian
        message.data[5] = (temp >> 8) & 0xff        #big endian
        message.data[7] = self.config['EXT_TEMP1_ID'] 
        message.data[8] = self.config['EXT_TEMP2_ID'] 
        message.data[9] = 0xFF
        message.data[10] = 0xFF

        while not self.can_fd_receive_queue.empty():
            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
        self.can_fd_transmitte_queue.put(message) 
        time.sleep(0.10)
        
        if not self.can_fd_receive_queue.empty():
            self.logging.info(f'id : {self.id + 1} unit board is initialized')    
            message = self.can_fd_receive_queue.get()
            self.logging.info(f'id : {self.id + 1} Received message: {message}')
            # if self.socket[0]:
            #     if message.data[4] == 1:
            #         self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"init success!"}), 'UTF-8'))
            #     else:
            #         self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"init fail!"}), 'UTF-8'))
        else:
            self.logging.warning(f'id : {self.id + 1} unit board is not response')      
        ############################################################################    
        global g_temp_control_start
                    
        while True:
            try: 
                command = self.command_queue.get()
                self.i2c_semaphor.acquire()
                i2cbus = smbus.SMBus(1)
                if command['unit_id'] < 8:
                    i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF & (~command['unit_id']))
                else:
                    i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF & (~command['unit_id']))
                if not command:
                    self.logging.warning(f'id : {self.id + 1} Timeout waiting for command')
                else: 
                    if command['cmd'] == 'SET_MOTOR':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x11, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        temp = int(command['speed'])
                        message.data[5] = temp & 0xff               #big endian
                        message.data[4] = (temp >> 8) & 0xff        #big endian
                        if command['dir'] == 'FW':
                            message.data[6] = 1
                        else:
                            message.data[6] = 0
                        if command['onoff'] == 'ON':
                            message.data[7] = 1
                        else:
                            message.data[7] = 0
                            
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                        
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.40)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'id : {self.id + 1} Received message: {message}')
                            if self.socket[0]:
                                if message.data[4] == 1:
                                    self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":f"success!"}), 'UTF-8'))
                                else:
                                    self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"fail!"}), 'UTF-8'))
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response')               
                    elif command['cmd'] == 'GET_ADC':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
            
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.05)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'id : {self.id + 1} Received message: {message}')
                            self.unit_semaphor.acquire()
                            self.shared_memory[0 + self.id*20] = (np.int32)((np.int32)(message.data[5] << 8) | (np.int32)(message.data[4]))
                            self.shared_memory[1 + self.id*20]  = (np.int32)((np.int32)(message.data[7] << 8) | (np.int32)(message.data[6]))
                            self.shared_memory[2 + self.id*20] = (np.int32)((np.int32)(message.data[9] << 8) | (np.int32)(message.data[8]))
                            self.shared_memory[3 + self.id*20] = (np.int32)((np.int32)(message.data[11] << 8) | (np.int32)(message.data[10]))
                            self.shared_memory[4 + self.id*20] = (np.int32)((np.int32)(message.data[13] << 8) | (np.int32)(message.data[12]))
                            self.shared_memory[5 + self.id*20] = (np.int32)((np.int32)(message.data[15] << 8) | (np.int32)(message.data[14]))
                            self.unit_semaphor.release()
                            if self.socket[0]:
                                self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!", 
                                                                    "ADC0": f'{self.shared_memory[0 + self.id*20]}',
                                                                    "ADC1": f'{self.shared_memory[1 + self.id*20]}',
                                                                    "ADC2": f'{self.shared_memory[2 + self.id*20]}',
                                                                    "ADC3": f'{self.shared_memory[3 + self.id*20]}',
                                                                    "ADC4": f'{self.shared_memory[4 + self.id*20]}',
                                                                    "ADC5": f'{self.shared_memory[5 + self.id*20]}'}), 'UTF-8'))
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response') 
                    elif command['cmd'] == 'SET_GPIO':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x13, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        message.data[4] = len(command['num'])
                        for i in range(message.data[4]):
                            message.data[5 + i] = command['value'][i]
                        
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
            
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.05)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'id : {self.id + 1} Received message: {message}')
                            if self.socket[0]:
                                self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!"}), 'UTF-8'))
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response') 
                    elif command['cmd'] == 'GET_STATUS':
                        # 온도계산 전에 GET_ADC를 호출 함.
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
            
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.05)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'id : {self.id + 1} Received message: {message}')
                            self.unit_semaphor.acquire()
                            self.shared_memory[0 + self.id*20] = (np.int32)((np.int32)(message.data[5] << 8) | (np.int32)(message.data[4]))
                            self.shared_memory[1 + self.id*20] = (np.int32)((np.int32)(message.data[7] << 8) | (np.int32)(message.data[6]))
                            self.shared_memory[2 + self.id*20] = (np.int32)((np.int32)(message.data[9] << 8) | (np.int32)(message.data[8]))
                            self.shared_memory[3 + self.id*20] = (np.int32)((np.int32)(message.data[11] << 8) | (np.int32)(message.data[10]))
                            self.shared_memory[4 + self.id*20] = (np.int32)((np.int32)(message.data[13] << 8) | (np.int32)(message.data[12]))
                            self.shared_memory[5 + self.id*20] = (np.int32)((np.int32)(message.data[15] << 8) | (np.int32)(message.data[14]))
                            self.unit_semaphor.release()
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response') 
                        # GET_ADC후에 GET_STATUS 수행
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x14, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                        
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.05)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'id : {self.id + 1} Received message: {message}')
                            self.unit_semaphor.acquire()
                            self.shared_memory[6 + self.id*20] = (np.int32)(message.data[7] << 24 | message.data[6] << 16 
                                                            | message.data[5] << 8 | message.data[4])
                            self.shared_memory[7 + self.id*20] = (np.int32)(message.data[9] << 8 | message.data[8])
                            self.shared_memory[8 + self.id*20] = (np.int32)(message.data[13] << 24 | message.data[12] << 16 
                                                            | message.data[11] << 8 | message.data[10])
                            self.shared_memory[9 + self.id*20] = (np.int32)(message.data[14])
                            self.shared_memory[10 + self.id*20] = (np.int32)(message.data[15] << 8 | message.data[16]) #RPM
                            self.shared_memory[11 + self.id*20] = (np.int32)(message.data[17] << 8 | message.data[18]) #load cell
                            
                            self.shared_memory[12 + self.id*20] = (np.int32)(message.data[19] << 8 | message.data[20]) #Ext Temp1
                            self.shared_memory[13 + self.id*20] = (np.int32)(message.data[21] << 8 | message.data[22]) #Ext Humi1
                            self.shared_memory[14 + self.id*20] = (np.int32)(message.data[23] << 8 | message.data[24]) #Ext Temp2
                            self.shared_memory[15 + self.id*20] = (np.int32)(message.data[25] << 8 | message.data[26]) #Ext Humi2
                            self.unit_semaphor.release()
                            if self.socket[0]:
                                inclination1 = 77.5 / (float(self.config['TEMP1_77_5']) - float(self.config['TEMP1_0']))
                                y_offset1 = inclination1 * float(self.config['TEMP1_0'])
                                
                                inclination2 = 77.5 / (float(self.config['TEMP2_77_5']) - float(self.config['TEMP2_0']))
                                y_offset2 = inclination2 * float(self.config['TEMP2_0'])
                                self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!", 
                                                                    "TEMP1" : f'{inclination1 * self.shared_memory[0 + self.id*20] - y_offset1 : 0.2F}',
                                                                    "TEMP2" : f'{inclination2 * self.shared_memory[1 + self.id*20] - y_offset2 : 0.2F}',
                                                                    "GPO4~GPO1": f'{self.shared_memory[6 + self.id*20]}',
                                                                    "GPO8~GPO5": f'{self.shared_memory[7 + self.id*20]}',
                                                                    "GPI4~GPI1": f'{self.shared_memory[8 + self.id*20]}',
                                                                    "GPI8~GPI5": f'{self.shared_memory[9 + self.id*20]}',
                                                                    "RPM": f'{self.shared_memory[10 + self.id*20]}',
                                                                    "LOAD CELL": f'{self.shared_memory[11 + self.id*20] * 0.01 : 0.2F}kg',
                                                                    "EXTTEMP1": f'{self.shared_memory[12 + self.id*20] * 0.01 : 0.2F}C',
                                                                    "EXTHUMI1": f'{self.shared_memory[13 + self.id*20] * 0.01 : 0.2F}%',
                                                                    "EXTTEMP2": f'{self.shared_memory[14 + self.id*20] * 0.01 : 0.2F}C',
                                                                    "EXTHUMI2": f'{self.shared_memory[15 + self.id*20] * 0.01 : 0.2F}%',
                                                                    }), 'UTF-8'))
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response') 
                    elif command['cmd'] == 'START_TEMP':
                        self.event.set()
                        g_temp_control_start = True
                        
                        if self.socket[0]:
                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!"}), 'UTF-8'))
                    elif command['cmd'] == 'STOP_TEMP':
                        g_temp_control_start = False
                        
                        if self.socket[0]:
                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!"}), 'UTF-8'))
                                 
                i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF)
                i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF)
                i2cbus.close()
                self.i2c_semaphor.release()
            except Exception as e:
                print(e)
                self.logging.error(f'id : {self.id + 1} {e}')
    # def pid_update(self):
    #     global _dt, _min, _max, _sv, _kp, _ki, _kd
    #     _dt, _min, _max, _sv, _kp, _ki, _kd = (5, 0., 5., 21., 0.1, 0.5, 0.01)