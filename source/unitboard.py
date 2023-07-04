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
from pid_controller import PID_COSMO_M
import csv
from konfig import Config
import configparser

setpoint = 15.0  # Target temperature
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.1  # Derivative gain

g_file_path = "./data/JSON_Ref_Stage101.txt"
g_temp_control_start = False
                   
class UnitBoardTempControl(threading.Thread):
    def __init__(self, id, queue, lock, event, logging, can_fd_transmitte_queue, 
                 can_fd_receive_queue, command_queue, shared_memory, unit_semaphor, config, shared_memory_size):
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
        self.pid = PID_COSMO_M(Kp, Ki, Kd, setpoint=setpoint)
        # Set output limits (heating/cooling power)
        self.pid.output_limits = (0, 50)
        self.pid.sample_time = 5  # Update every 0.01 seconds

        self.shared_memory = shared_memory
        self.shared_memory_size = shared_memory_size
        self.time_to_on = 0
        self.pid_call_time = 0
        self.unit_semaphor = unit_semaphor
        
        self.current_temp1 = 0              # 상부
        self.current_temp2 = 0              # 하부
        self.config = config
        self.ref_temp = 0
        self.ref_rpm = 0
        self.ref_motor_time = False
        self.check_time = 0
        self.cold_valve_status = 0
        
        with open(g_file_path, 'r') as file:
            self.ref_file = json.load(file)
            self.ref_id = self.ref_file['ID']
            self.ref_stage = self.ref_file['STAGE']
            self.ref_step = self.ref_file['STEP']
            self.ref_data = self.ref_file['DATA']
            self.ref_total  = len(self.ref_data)
            self.logging.info(f'id : {self.id + 1} UnitBoard read reference temperature data')
    
    def set_cold_valve(self, value):
        self.cold_valve_status = value
        
    def pid_task(self):    
        if self.pid_call_time:
            x = self.config["SOLVALVE2"]
            
            if self.time_to_on:
                self.time_to_on -= 1
                if self.cold_valve_status == 0:
                    message = {"UNIT_ID" : self.id + 1,                  
                                "CMD":"TEMP_VALVE",
                                "CHANNEL": x,
                                "VALUE" : True}
                    self.set_cold_valve(1)
                    self.command_queue.put(message) 
            else:
                if self.cold_valve_status == 1:
                    message = {"UNIT_ID" : self.id + 1,
                                "CMD":"TEMP_VALVE",
                                "CHANNEL": x,
                                "VALUE" : False} 
                    self.set_cold_valve(0) 
                    self.command_queue.put(message)
            self.pid_call_time -= 1
            Timer(0.1, self.pid_task).start()
        else:
            print(f'self.id {self.id+1} pid_task{self.pid_call_time} and {self.time_to_on} \
              CT {self.current_temp2:0.2F} and FT and {self.ref_temp}')
            Timer(0.1, self.pid_task).cancel()
            self.pid_event.set()
            
    def run(self):
            self.logging.info(f'id : {self.id + 1} UnitBoard Temp Control Thread Run')
            while True:
                try:
                    self.event.wait()
                    self.event.clear()
                    # self.pid.reset() 테스트 필요
                    ##############################################################################################################
                    ## 온도 관련 동작
                    ## 20230609
                    ## @K2H
                    ## 온도 테스트를 위한 데이저 저장. 
                    try:
                        self.writer_csv = open(f"pid_process{self.id+1}.csv", 'w', encoding='utf-8', newline='')
                        self.writer = csv.writer(self.writer_csv, delimiter=',')
                        self.writer.writerow(['time'] + ['ref.temp'] + ['real temp'] + ['valve on time'])   
                    except Exception as e:
                        print(e)
                    ##############################################################################################################

                    for x in range(self.ref_total):
                        if g_temp_control_start:
                            time_start = time.time()
                            self.ref_temp = self.ref_data[x]["TEMP"]
                            self.ref_rpm = self.ref_data[x]["M_RPM"]
                            self.ref_motor_time = self.ref_data[x]["M_TIME"]
                            self.pid.modify_setpoint(self.ref_temp)
                            
                            message = {"UNIT_ID" : self.id + 1,                  
                                        "CMD":"TEMP_RPM",
                                        "SPEED" : self.ref_rpm, 
                                        "DIR"   : 'FW',            #FW = forward, RV = reverse
                                        "ONOFF" : 'ON', 
                                        "TIME" : self.ref_motor_time}    
                            self.command_queue.put(message) 
                            self.logging.info(f'id : {self.id + 1} UnitBoard Temp Control Thread {x} Step Start at {time_start} Time')
                            
                            while (time_start + self.ref_step) > time.time():
                                if g_temp_control_start:
                                    # client.py에서 data = {"unit_id" : x + 1, "cmd":"GET_STATUS", "send" : False, "raw" : False}
                                    # 로 데이터를 보내므로 온도 값이 계산되어 저장됨 따라서 *0.01을 하면 온도 값으로 사용
                                    self.current_temp2 = self.shared_memory[0x11 + self.id*self.shared_memory_size] * 0.01 #온도 센서 2
                                    
                                    inc = self.pid(self.current_temp2)
                                    self.time_to_on = round(inc)                            #소수점 첫번째에서 반올림
                                    self.pid_call_time = 49                                 #타이머 호출 회수 -1
                                    self.writer.writerow([time.time(), self.ref_temp, self.current_temp2, self.time_to_on])  
                                    self.timer = Timer(0.1, self.pid_task).start()
                                    
                                    self.pid_event.wait()                                   #1초 타이머가 5번 호출되는것을 기다림
                                    self.pid_event.clear()
                                else:
                                    self.writer_csv.close() 
                                    break
                        else:
                            self.pid.reset()
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
        
    def unit_process(self, n, shm, arr, semaphor, receive_queue, command_queue):
        self.new_shm = shared_memory.SharedMemory(name=shm)
        self.shared_memory = np.ndarray(arr.shape, dtype=arr.dtype, buffer=self.new_shm.buf)
        self.logging.info(f'Process {os.getpid()} and {n} are created')  
        self.id = n
        self.unit_semaphor = semaphor
        self.can_fd_receive_queue = receive_queue
        self.command_queue = command_queue
        self.event = threading.Event()
        self.lock = threading.Lock()
        self.queue = Queue(12)
        
        try:
            # main_func.config_file = Config("/home/pi/Projects/cosmo-m/config/config.ini")      # For VSC
            # common_config = main_func.config_file.get_map('common')
            self.config_file = configparser.ConfigParser()  ## 클래스 객체 생성
            self.config_file.read('/home/pi/Projects/cosmo-m/config/config.ini')  ## 파일 읽기        
            common_config = self.config_file['common']
            self.shared_memory_size = int(common_config['SHARED_MEMORY_SIZE'])
            self.config = self.config_file[f'unit_board{self.id+1}']
        except Exception as e:
            print(e)
        self.shared_memory[0x1D + self.id * self.shared_memory_size] = os.getpid()       
        # 온도조절 관련 쓰레드 생성 ##################################################
        temp_thread = UnitBoardTempControl(self.id, self.queue, self.lock, self.event, self.logging, self.can_fd_transmitte_queue, 
                                                           self.can_fd_receive_queue, self.command_queue, 
                                                           self.shared_memory, self.unit_semaphor, self.config, self.shared_memory_size)
        temp_thread.start()
        ######################################################################################################################################################
        # 처음 부팅이 되면 환경 설정을 유닛보드로 전송 ################################
        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x16, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
        message.data[4] = int(self.config['MOTOR_ID'])
        temp = int(self.config['SLEEP_SPEED'])
        message.data[6] = temp & 0xff               #big endian
        message.data[5] = (temp >> 8) & 0xff        #big endian
        message.data[7] = int(self.config['EXT_TEMP1_ID']) 
        message.data[8] = int(self.config['EXT_TEMP2_ID']) 
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
        else:
            self.logging.warning(f'id : {self.id + 1} unit board is not response')      
        ######################################################################################################################################################    
        global g_temp_control_start
                    
        while True:
            try: 
                command = self.command_queue.get()
                self.i2c_semaphor.acquire()
                i2cbus = smbus.SMBus(1)
                if command['UNIT_ID'] < 8:
                    i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF & (~command['UNIT_ID']))
                else:
                    i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF & (~command['UNIT_ID']))
                if not command:
                    self.logging.warning(f'id : {self.id + 1} Timeout waiting for command')
                else: 
                    if command['CMD'] == 'REF':
                        pass
                    elif command['CMD'] == 'STATE':
                        if int(self.config_file['TANK_ID']) == command['DATA'][self.id]['TANK_ID']:
                            if command['DATA'][self.id]['STATUS'] == 'None':
                                status = 0
                            elif command['DATA'][self.id]['STATUS'] == 'Stop':
                                status = 1
                            elif command['DATA'][self.id]['STATUS'] == 'Run':
                                status = 2
                            elif command['DATA'][self.id]['STATUS'] == 'Puase':
                                status = 3
                            elif command['DATA'][self.id]['STATUS'] == 'Initial':
                                status = 4
                            elif command['DATA'][self.id]['STATUS'] == 'Error':
                                status = 5
                            self.shared_memory[0x18 + self.id*self.shared_memory_size] = command['DATA'][self.id]['STAGE'] << 16 | status
                            
                    elif command['CMD'] == 'SET_MOTOR':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x11, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        temp = int(command['SPEED'])
                        message.data[5] = temp & 0xff               # big endian
                        message.data[4] = (temp >> 8) & 0xff        # big endian
                        if command['DIR'] == 'FW':
                            message.data[6] = 1
                        else:
                            message.data[6] = 0
                        if command['ONOFF'] == 'ON':
                            message.data[7] = 1
                        else:
                            message.data[7] = 0
                            
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                        
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.40)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            if message.data[1] == 0x11:
                                self.logging.info(f'id : {self.id + 1} Received message: {message}')
                                if self.socket[0]:
                                    if message.data[4] == 1:
                                        self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":f"success!"}), 'UTF-8'))
                                    else:
                                        self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"fail!"}), 'UTF-8'))
                            else:
                                self.logging.warning(f'id : {self.id + 1} unit board is wrong response')  
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response')               
                    elif command['CMD'] == 'GET_ADC':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
            
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.06)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            if message.data[1] == 0x12:
                                self.logging.info(f'id : {self.id + 1} Received message: {message}')
                                self.unit_semaphor.acquire()
                                self.shared_memory[0x00 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[5] << 8) | (np.int32)(message.data[4]))
                                self.shared_memory[0x01 + self.id*self.shared_memory_size]  = (np.int32)((np.int32)(message.data[7] << 8) | (np.int32)(message.data[6]))
                                self.shared_memory[0x02 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[9] << 8) | (np.int32)(message.data[8]))
                                self.shared_memory[0x03 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[11] << 8) | (np.int32)(message.data[10]))
                                self.shared_memory[0x04 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[13] << 8) | (np.int32)(message.data[12]))
                                self.shared_memory[0x05 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[15] << 8) | (np.int32)(message.data[14]))
                                self.unit_semaphor.release()
                                if self.socket[0]:
                                    self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!", 
                                                "ADC0": f'{self.shared_memory[0x00 + self.id*self.shared_memory_size]}',
                                                "ADC1": f'{self.shared_memory[0x01 + self.id*self.shared_memory_size]}',
                                                "ADC2": f'{self.shared_memory[0x02 + self.id*self.shared_memory_size]}',
                                                "ADC3": f'{self.shared_memory[0x03 + self.id*self.shared_memory_size]}',
                                                "ADC4": f'{self.shared_memory[0x04 + self.id*self.shared_memory_size]}',
                                                "ADC5": f'{self.shared_memory[0x05 + self.id*self.shared_memory_size]}'}), 'UTF-8'))
                            else:
                                self.logging.warning(f'id : {self.id + 1} unit board is wrong response') 
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response') 
                    elif command['CMD'] == 'SET_GPIO':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x13, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        message.data[4] = len(command['NUM'])
                        for i in range(message.data[4]):
                            message.data[5 + i] = command['VALUE'][i]
                        
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
            
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.06)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            if message.data[1] == 0x13:
                                self.logging.info(f'id : {self.id + 1} Received message: {message}')
                                if self.socket[0]:
                                    self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!"}), 'UTF-8'))
                            else:
                                self.logging.warning(f'id : {self.id + 1} unit board is wrong response') 
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response') 
                    elif command['CMD'] == 'GET_STATUS':
                        # 온도계산 전에 GET_ADC를 호출 함.
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
            
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.06)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            if message.data[1] == 0x12:
                                if command['SEND']:                 # 로고에 너무 많이 쌓이는 데이터 방지, 
                                    self.logging.info(f'id : {self.id + 1} Received message: {message}')
                                self.unit_semaphor.acquire()
                                self.shared_memory[0x00 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[5] << 8) | (np.int32)(message.data[4]))
                                self.shared_memory[0x01 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[7] << 8) | (np.int32)(message.data[6]))
                                self.shared_memory[0x02 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[9] << 8) | (np.int32)(message.data[8]))
                                self.shared_memory[0x03 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[11] << 8) | (np.int32)(message.data[10]))
                                self.shared_memory[0x04 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[13] << 8) | (np.int32)(message.data[12]))
                                self.shared_memory[0x05 + self.id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[15] << 8) | (np.int32)(message.data[14]))
                                self.unit_semaphor.release()
                            else:
                                self.logging.warning(f'id : {self.id + 1} unit board is wrong response')
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response') 
                        # GET_ADC후에 GET_STATUS 수행
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x14, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                        
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.06)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            if message.data[1] == 0x14:
                                if command['SEND']:                      # 로고에 너무 많이 쌓이는 데이터 방지, 
                                    self.logging.info(f'id : {self.id + 1} Received message: {message}')
                                inclination1 = 77.5 / (float(self.config['TEMP1_77_5']) - float(self.config['TEMP1_0']))
                                y_offset1 = inclination1 * float(self.config['TEMP1_0'])
                                
                                inclination2 = 77.5 / (float(self.config['TEMP2_77_5']) - float(self.config['TEMP2_0']))
                                y_offset2 = inclination2 * float(self.config['TEMP2_0'])
                                
                                # inclination3 = 77.5 / (float(self.config['TEMP3_77_5']) - float(self.config['TEMP3_0']))
                                # y_offset3 = inclination3 * float(self.config['TEMP3_0'])
                                
                                # inclination4 = 77.5 / (float(self.config['TEMP4_77_5']) - float(self.config['TEMP4_0']))
                                # y_offset4 = inclination4 * float(self.config['TEMP4_0'])
                                    
                                self.unit_semaphor.acquire()
                                self.shared_memory[0x06 + self.id*self.shared_memory_size] = (np.int32)(message.data[7] << 24 | message.data[6] << 16 
                                                                | message.data[5] << 8 | message.data[4])
                                self.shared_memory[0x07 + self.id*self.shared_memory_size] = (np.int32)(message.data[9] << 8 | message.data[8])
                                self.shared_memory[0x08 + self.id*self.shared_memory_size] = (np.int32)(message.data[13] << 24 | message.data[12] << 16 
                                                                | message.data[11] << 8 | message.data[10])
                                self.shared_memory[0x09 + self.id*self.shared_memory_size] = (np.int32)(message.data[14])
                                self.shared_memory[0x0A + self.id*self.shared_memory_size] = (np.int32)(message.data[15] << 8 | message.data[16]) #RPM
                                self.shared_memory[0x0B + self.id*self.shared_memory_size] = (np.int32)(message.data[17] << 8 | message.data[18]) #load cell
                                
                                self.shared_memory[0x0C + self.id*self.shared_memory_size] = (np.int32)(message.data[19] << 8 | message.data[20]) #Ext Temp1
                                self.shared_memory[0x0D + self.id*self.shared_memory_size] = (np.int32)(message.data[21] << 8 | message.data[22]) #Ext Humi1
                                self.shared_memory[0x0E + self.id*self.shared_memory_size] = (np.int32)(message.data[23] << 8 | message.data[24]) #Ext Temp2
                                self.shared_memory[0x0F + self.id*self.shared_memory_size] = (np.int32)(message.data[25] << 8 | message.data[26]) #Ext Humi2
                                self.unit_semaphor.release()
                                
                                self.shared_memory[0x10 + self.id*self.shared_memory_size] = float(f'{(inclination1 * self.shared_memory[0 + self.id*self.shared_memory_size] - y_offset1) * 100 : 0.2F}')
                                self.shared_memory[0x11 + self.id*self.shared_memory_size] = float(f'{(inclination2 * self.shared_memory[1 + self.id*self.shared_memory_size] - y_offset2) * 100 : 0.2F}')
                                # self.shared_memory[0x12 + self.id*self.shared_memory_size] = float(f'{(inclination3 * self.shared_memory[2 + self.id*self.shared_memory_size] - y_offset3) * 100 : 0.2F}')
                                # self.shared_memory[0x13 + self.id*self.shared_memory_size] = float(f'{(inclination4 * self.shared_memory[3 + self.id*self.shared_memory_size] - y_offset4) * 100 : 0.2F}')
                                    
                                if command['SEND'] and self.socket[0]:
                                    self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!", 
                                        "TEMP1" : f'{self.shared_memory[0x10 + self.id*self.shared_memory_size] * 0.01: 0.2F}',
                                        "TEMP2" : f'{self.shared_memory[0x11 + self.id*self.shared_memory_size] * 0.01: 0.2F}',
                                        "GPO4~GPO1": f'{self.shared_memory[0x06 + self.id*self.shared_memory_size]}',
                                        "GPO8~GPO5": f'{self.shared_memory[0x07 + self.id*self.shared_memory_size]}',
                                        "GPI4~GPI1": f'{self.shared_memory[0x08 + self.id*self.shared_memory_size]}',
                                        "GPI8~GPI5": f'{self.shared_memory[0x09 + self.id*self.shared_memory_size]}',
                                        "RPM": f'{self.shared_memory[0x0A + self.id*self.shared_memory_size]}',
                                        "LOAD CELL": f'{self.shared_memory[0x0B + self.id*self.shared_memory_size] * 0.01 : 0.2F}kg',
                                        "EXTTEMP1": f'{self.shared_memory[0x0C + self.id*self.shared_memory_size] * 0.01 : 0.2F}C',
                                        "EXTHUMI1": f'{self.shared_memory[0x0D + self.id*self.shared_memory_size] * 0.01 : 0.2F}%',
                                        "EXTTEMP2": f'{self.shared_memory[0x0E + self.id*self.shared_memory_size] * 0.01 : 0.2F}C',
                                        "EXTHUMI2": f'{self.shared_memory[0x0F + self.id*self.shared_memory_size] * 0.01 : 0.2F}%',
                                        }), 'UTF-8'))
                            else:
                                self.logging.warning(f'id : {self.id + 1} unit board is wrong response')
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response') 
                    elif command['CMD'] == 'START_TEMP':
                        self.event.set()
                        g_temp_control_start = True
                        
                        if self.socket[0]:
                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!"}), 'UTF-8'))
                    elif command['CMD'] == 'STOP_TEMP':
                        g_temp_control_start = False
                        
                        if self.socket[0]:
                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"success!"}), 'UTF-8'))
                    elif command['CMD'] == 'TEMP_RPM':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x17, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])

                        temp = int(command['SPEED'])
                        message.data[5] = temp & 0xff               # big endian
                        message.data[4] = (temp >> 8) & 0xff        # big endian
                        if command['DIR'] == 'FW':
                            message.data[6] = 1
                        else:
                            message.data[6] = 0
                        if command['ONOFF'] == 'ON':
                            message.data[7] = 1
                        else:
                            message.data[7] = 0
                        
                        temp = int(command['TIME'])
                        message.data[9] = temp & 0xff               # big endian
                        message.data[8] = (temp >> 8) & 0xff        # big endian
                             
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                        
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.40)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            if message.data[1] == 0x17:
                                self.logging.info(f'id : {self.id + 1} Received message: {message}')
                                if self.socket[0]:
                                    if message.data[4] == 1:
                                        self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":f"success!"}), 'UTF-8'))
                                    else:
                                        self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"fail!"}), 'UTF-8'))
                            else:
                                self.logging.warning(f'id : {self.id + 1} unit board is wrong response')  
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response')   
                    elif command['CMD'] == 'TEMP_VALVE':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x18, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])

                        message.data[4] = int(command['CHANNEL'])             
                        message.data[5] = command['VALUE']
                             
                        while not self.can_fd_receive_queue.empty():
                            self.can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                        
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.06)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            if message.data[1] == 0x18:
                                self.logging.info(f'id : {self.id + 1} Received message: {message}')
                                temp_thread.set_cold_valve(message.data[5])
                                
                                if self.socket[0]:
                                    if message.data[4] == 1:
                                        self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":f"success!"}), 'UTF-8'))
                                    else:
                                        self.socket[0].sendall(bytes(json.dumps({"id" : f'{self.id + 1}', "status":"fail!"}), 'UTF-8'))
                            else:
                                self.logging.warning(f'id : {self.id + 1} unit board is wrong response')  
                        else:
                            self.logging.warning(f'id : {self.id + 1} unit board is not response')             
                i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF)
                i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF)
                i2cbus.close()
                self.i2c_semaphor.release()
            except Exception as e:
                print(e)
                i2cbus.close()
                self.i2c_semaphor.release()
                self.unit_semaphor.release()
                self.logging.error(f'id : {self.id + 1} {e}')