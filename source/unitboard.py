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
import configparser
import traceback

setpoint = 15.0  # Target temperature
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.1  # Derivative gain

ON = 1
OFF = 0
# g_file_path = "./data/JSON_Ref_Stage101.txt"
                   
class UnitBoardTempControl(threading.Thread):
    def __init__(self, id, event, logging, can_fd_transmitte_queue, 
                 command_queue, shared_memory, unit_semaphor, config, shared_memory_size):
        threading.Thread.__init__(self)
        self.daemon = True
        self.id = id                            # id는 0부터 시작
        self.logging = logging
        self.can_fd_transmitte_queue = can_fd_transmitte_queue
        self.event = event
        self.pid_timer_event = threading.Event()
        self.command_queue = command_queue
        self.pid = PID_COSMO_M(Kp, Ki, Kd, setpoint=setpoint)
        # Set output limits (heating/cooling power)
        self.pid.output_limits = (0, 50)
        self.pid.sample_time = 5  # Update every 0.01 seconds

        self.shared_memory_u = shared_memory
        self.shared_memory_size = shared_memory_size
        self.time_to_on = 0                 # Valve 릴레이가 ON되는 시간 변수
        self.pid_timer_call_time = 0        # pid or timer 계산 시간 5초. 0.1초 단위이므로 50
        self.unit_semaphor = unit_semaphor
        
        self.config = config
        
        self.check_time = 0
        self.cold_valve_status = 0
        self.temp_control_start = False

        self.ref_datas = []                 # 서버에서 전송되는 REF 데이터 모든 것을 저장하는 리스트
        self.ref_stage = 0
        self.ref_step = 0
        self.ref_data = []                  # REF 데이터 중에서 data항목 저장
        self.ref_total = 0
    
        self.file_write = False             # CSV 파일을 새로 만들지 결정
        self.file_write_state = None        # CSV 파일 저장 조건 상태
        self.file_index = 0
        self.timer_control_valve = False    # 타이머로 제어 할 때, 시간 마다 한 번만 제어 하기위한 변수
        self.motor_rpm = 0                  # 모터 현재 속도
        
    def set_cold_valve(self, value):
        self.cold_valve_status = value
        x = self.config["SOLVALVE2"]        #냉각수 밸브 I/O 번호
        message = {"UNIT_ID" : self.id,                  
                    "CMD":"TEMP_VALVE",
                    "CHANNEL": x,
                    "VALUE" : value}
        self.command_queue.put(message) 
    
    def pid_task(self):    
        if self.pid_timer_call_time > 0 and self.temp_control_start:            
            if self.time_to_on:
                self.time_to_on -= 1
                if self.cold_valve_status == 0:
                    self.set_cold_valve(ON)
            else:
                if self.cold_valve_status == 1:
                    self.set_cold_valve(OFF) 
            self.pid_timer_call_time -= 1
            Timer(0.1, self.pid_task).start()
        else:
            Timer(0.1, self.pid_task).cancel()
            self.pid_timer_event.set()
            
    def timer_task(self):    
        if self.pid_timer_call_time > 0 and self.temp_control_start:            
            if self.time_to_on > 0 and self.timer_control_valve:
                self.time_to_on -= 1
                if self.cold_valve_status == 0:
                    self.set_cold_valve(ON)
                    # 온도 제어할 때, 모터가 100rpm보다 느린상태라면 구동 시킴
                    ref_rpm = int(self.config["TEMP_CONTROL_MOTOR_RPM"])
                    ref_motor_time = int(self.config["TEMP_CONTROL_MOTOR_TIME"])
                    if self.motor_rpm < 100:                      
                        message = {"UNIT_ID" : self.id,                  
                                        "CMD":"TEMP_RPM",
                                        "SPEED" : ref_rpm, 
                                        "DIR"   : 'FW',            #FW = forward, RV = reverse
                                        "ONOFF" : 'ON', 
                                        "TIME" : ref_motor_time,
                                        "SEND" : False}    
                        self.command_queue.put(message) 
                        self.logging.info(f"{message['CMD']} command is inserted Unit Board")
            else:
                if self.cold_valve_status == 1:
                    self.set_cold_valve(OFF) 
                    self.timer_control_valve = False
            self.pid_timer_call_time -= 1
            Timer(1, self.timer_task).start()
        else:
            Timer(1, self.timer_task).cancel()
            
            if self.cold_valve_status == 1:
                self.set_cold_valve(OFF) 
            # 한시간 마다 온도를 측정해서 냉각수를 구동시키기 때문에 위 if 문이 참이 아니면 다음 한시간을 기다리기 위해 아래
            # self.timer_control_valve = False를 수행함
            self.timer_control_valve = False
            self.pid_timer_event.set()
              
    def run(self):
            self.logging.info(f'id : {self.id} UnitBoard Temp Control Thread Run')
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
                    if self.file_write:
                        try:
                            self.writer_csv = open(f"/home/pi/Projects/cosmo-m/data/pid_process{self.id}_{self.file_index}.csv", 'w', encoding='utf-8', newline='')
                            self.writer = csv.writer(self.writer_csv, delimiter=',')
                            self.writer.writerow(['time'] + ['ref.temp'] + ['real temp'] + ['valve on time']  + ['ext1 temp'] + ['ext1 humi'] + ['ext2 temp'] + ['ext2 humi'])   
                            self.file_write = False
                        except Exception as e:
                            print(e)
                    ##############################################################################################################
                    for x in range(self.ref_total):
                        if self.temp_control_start:
                            time_start = time.time()
                            ref_temp = float(self.ref_data[x]["TEMP"])
                            ref_rpm = int(self.ref_data[x]["M_RPM"])
                            ref_motor_time = int(self.ref_data[x]["M_TIME"])
                            ref_temp_error = float(self.ref_data[x]["TEMP_MGN"])
                            self.pid.modify_setpoint(ref_temp)
                            
                            # 2023-08-11 테스트용으로 모터 RPM을 1000으로 수정
                            # ref_rpm = 1000
                            #################################################
                            message = {"UNIT_ID" : self.id,                  
                                        "CMD":"TEMP_RPM",
                                        "SPEED" : ref_rpm, 
                                        "DIR"   : 'FW',            #FW = forward, RV = reverse
                                        "ONOFF" : 'ON', 
                                        "TIME" : ref_motor_time,
                                        "SEND" : False}    
                            self.command_queue.put(message) 
                            self.logging.info(f"{message['CMD']} command is inserted Unit Board")
                            # self.logging.info(f'id : {self.id} UnitBoard Temp Control Thread {x} Step Start at {time_start} Time')
                            self.timer_control_valve = True     # ref_step마다 한번씩 ON 해준다.
                            
                            while (time_start + self.ref_step) > time.time():
                                if self.temp_control_start:
                                    # client.py에서 data = {"unit_id" : x , "cmd":"GET_STATUS", "send" : False, "raw" : False}
                                    # 로 데이터를 보내므로 온도 값이 계산되어 저장됨 따라서 *0.01을 하면 온도 값으로 사용
                                    current_temp2 = self.shared_memory_u[0x11 + self.id*self.shared_memory_size] * 0.01 #온도 센서 2
                                    current_ext_temp1 = self.shared_memory_u[0x0C + self.id*self.shared_memory_size] * 0.1 #Ext Temp1
                                    current_ext_humi1 = self.shared_memory_u[0x0D + self.id*self.shared_memory_size] * 0.1 #Ext Humi1
                                    current_ext_temp2 = self.shared_memory_u[0x0E + self.id*self.shared_memory_size] * 0.1 #Ext Temp2
                                    current_ext_humi2 = self.shared_memory_u[0x0F + self.id*self.shared_memory_size] * 0.1 #Ext Humi2
                                    self.motor_rpm = self.shared_memory_u[0x0A + self.id*self.shared_memory_size]          #RPM
                                    
                                    if self.file_write_state:                               #STATE가 Run이면 True Pause면 False
                                        self.writer.writerow([time.time(), ref_temp, current_temp2, self.time_to_on, current_ext_temp1, current_ext_humi1, 
                                                              current_ext_temp2, current_ext_humi2])
                                        print(f'id: {self.id} period: {time.time()} time to on: {self.time_to_on} C.T: {current_temp2:0.2F} and F.T: {ref_temp}') 
                                        
                                    if self.config["TEMP_CONTROL"] == 'PID':
                                        inc = self.pid(current_temp2)
                                        self.time_to_on = round(inc)                        #소수점 첫번째에서 반올림
                                        self.pid_timer_call_time = 49                       #타이머 호출 회수 -1
                                        pid_t = Timer(0.1, self.pid_task).start()           #0.1초 타이머
                                    elif self.config["TEMP_CONTROL"] == 'TIMER':
                                        if ref_temp < current_temp2:
                                            self.time_to_on = int(self.config["TEMP_CONTROL_TIME"]) 
                                        else:
                                            self.time_to_on = 0
                                        self.pid_timer_call_time = 9  
                                        timer_t = Timer(1, self.timer_task).start()         #1초 타이머
                                        
                                    self.pid_timer_event.wait()                             #0.1초 또는 1초 타이머가 50번 또는 7 호출되는것을 기다림
                                    self.pid_timer_event.clear()
                                else:
                                    self.writer_csv.close() 
                                    break
                        else:
                            self.timer_control_valve = False
                            if self.pid_timer_call_time > 0:
                                self.pid_timer_call_time = 0
                            #################################################
                            #message = {"UNIT_ID" : self.id,                  
                            #            "CMD":"TEMP_RPM",
                            #            "SPEED" : 0, 
                            #            "DIR"   : 'FW',            #FW = forward, RV = reverse
                            #            "ONOFF" : 'OFF', 
                            #            "TIME" : 0,
                            #            "SEND" : False}    
                            #self.command_queue.put(message) 
                            #self.logging.info(f"{message['CMD']} command is inserted Unit Board")
                            break
                    self.pid.reset()            #pause 또는 stop이 오면 pid reset후 처음부터 다시 시작
                    self.set_cold_valve(OFF)    #pause 또는 stop이 오면 냉각 밸브를 off 시킴
                    ##############################################################################################################
                except Exception as e:
                    self.writer_csv.close()
                    print(e)

class UnitBoard:
    def __init__(self, transmitte_queue, shared_object, GPIOADDR, i2c_semaphor) -> None:
        self.can_fd_transmitte_queue = transmitte_queue
        self.socket = shared_object
        self.GPIOADDR = GPIOADDR
        self.i2c_semaphor = i2c_semaphor
        # self.pid_update()
        
    def unit_process(self, n, shm, arr, semaphor, receive_queue, cmd_queue, logging):
        new_shm = shared_memory.SharedMemory(name=shm)
        shared_memory_u = np.ndarray(arr.shape, dtype=arr.dtype, buffer=new_shm.buf)
        logging.info(f'Process {os.getpid()} and {n} are created')  
        id = n
        unit_semaphor = semaphor
        can_fd_receive_queue = receive_queue
        command_queue = cmd_queue
        event = threading.Event()
        old_status = "None"
        old_stage = 1000
        try:
            self.config_file = configparser.ConfigParser()  ## 클래스 객체 생성
            self.config_file.read('/home/pi/Projects/cosmo-m/config/config.ini')  ## 파일 읽기        
            common_config = self.config_file['common']
            self.shared_memory_size = int(common_config['SHARED_MEMORY_SIZE'])
            self.config = self.config_file[f'unit_board{id}']
        except Exception as e:
            logging.error(f'id : {id} config.ini file open error')
            print(e)
        shared_memory_u[0x1D + id * self.shared_memory_size] = os.getpid()       
        # 온도조절 관련 쓰레드 생성 ##################################################
        temp_thread = UnitBoardTempControl(id, event, logging, self.can_fd_transmitte_queue, 
                                                           command_queue, 
                                                           shared_memory_u, unit_semaphor, self.config, self.shared_memory_size)
        temp_thread.start()
        ######################################################################################################################################################
        # 처음 부팅이 되면 환경 설정을 유닛보드로 전송 ################################
        # SET_CONFIG 명령어 수행
        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+id,  
                                    data=[0xF2, 0x16, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
        message.data[4] = int(self.config['MOTOR_ID'])
        temp = int(self.config['SLEEP_SPEED'])
        message.data[6] = temp & 0xff               #big endian
        message.data[5] = (temp >> 8) & 0xff        #big endian
        message.data[7] = int(self.config['EXT_TEMP1_ID']) 
        message.data[8] = int(self.config['EXT_TEMP2_ID']) 
        message.data[9] = 1                         # 1 = GPIO초기화 
        message.data[10] = 0xFF

        while not can_fd_receive_queue.empty():
            can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
        
        self.can_fd_transmitte_queue.put(message) 
        time.sleep(0.10)

        if not can_fd_receive_queue.empty():
            logging.info(f'id : {id} unit board is initialized')    
            message = can_fd_receive_queue.get()
            fw_version = (message.data[4] << 8) | (message.data[5])
            logging.info(f'id : {id} firmware version is : {fw_version * 0.01 : 0.2F}')
        else:
            logging.warning(f'id : {id} unit board is not response')    
        ######################################################################################################################################################                        
        while True:
            try: 
                command = command_queue.get()
                self.i2c_semaphor.acquire()
                i2cbus = smbus.SMBus(1)
                if int(command['UNIT_ID']) < 14:
                    i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF & (~(int(command['UNIT_ID']) + 1)))
                else:
                    i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF & (~(int(command['UNIT_ID']) + 1)))
                self.i2c_semaphor.release()
                if not command:
                    logging.warning(f'id : {id} Timeout waiting for command')
                else: 
                    if command['CMD'] == 'REF':
                        if int(self.config['TANK_ID']) == int(command['TANK_ID']) and int(self.config['ADDRESS'], 16) != 0xFFF:
                            temp_thread.ref_datas.append(command)
                    elif command['CMD'] == 'STATE':
                        if int(self.config['TANK_ID']) == int(command['DATA'][id]['TANK_ID']) and int(self.config['ADDRESS'], 16) != 0xFFF:
                            if command['DATA'][id]['STATUS'] == 'None':
                                temp_thread.temp_control_start = False
                                shared_memory_u[0x18 + id*self.shared_memory_size] = int(command['DATA'][id]['STAGE']) << 16 | 0
                                status = 0
                            elif command['DATA'][id]['STATUS'] == 'Stop':
                                temp_thread.temp_control_start = False
                                shared_memory_u[0x18 + id*self.shared_memory_size] = int(command['DATA'][id]['STAGE']) << 16 | 0
                                status = 1
                            elif command['DATA'][id]['STATUS'] == 'Run':
                                if not temp_thread.temp_control_start:      #기존 동작하지 않고 있으면 동작 함.
                                    if len(temp_thread.ref_datas) > 0:
                                        ref_command = temp_thread.ref_datas.pop(0)
                                        temp_thread.ref_stage = int(ref_command['STAGE'])
                                        temp_thread.ref_step = int(ref_command['STEP'])
                                        temp_thread.ref_data = ref_command['DATA']
                                        temp_thread.ref_total = len(temp_thread.ref_data)
                                    else:
                                        logging.error(f'id : {id} reference data is empty')
                                    temp_thread.temp_control_start = True
                                    temp_thread.file_write = True
                                    temp_thread.file_write_state = True
                                    temp_thread.file_index += 1
                                    event.set()
                                shared_memory_u[0x18 + id*self.shared_memory_size] = int(command['DATA'][id]['STAGE']) << 16 | 0
                                status = 2
                            elif command['DATA'][id]['STATUS'] == 'Pause':
                                if temp_thread.temp_control_start:      #기존 온도 쓰레드가 끝나지 않으면 강제 종료
                                    temp_thread.temp_control_start = False
                                    time.sleep(5)                       #PID 쓰레드는 5초 단위로 동작하므로 기존 쓰레드가 끝날 때 까지 5초 대기
                                if len(temp_thread.ref_datas) > 0:
                                    ref_command = temp_thread.ref_datas.pop(0)
                                    temp_thread.ref_stage = int(ref_command['STAGE'])
                                    temp_thread.ref_step = int(ref_command['STEP'])
                                    temp_thread.ref_data = ref_command['DATA']
                                    temp_thread.ref_total = len(temp_thread.ref_data)
                                    
                                    temp_thread.temp_control_start = True
                                    temp_thread.file_write_state = False
                                    event.set()
                                else:
                                    logging.error(f'id : {id} reference data is empty')
                                shared_memory_u[0x18 + id*self.shared_memory_size] = int(command['DATA'][id]['STAGE']) << 16 | 0
                                status = 3
                            elif command['DATA'][id]['STATUS'] == 'Initial':
                                temp_thread.temp_control_start = False
                                temp_thread.file_index = 0
                                temp_thread.ref_datas.clear()
                                shared_memory_u[0x18 + id*self.shared_memory_size] = int(command['DATA'][id]['STAGE']) << 16 | 0
                                logging.info(f'id : {id} reference data status is  Initial')
                                status = 4
                            elif command['DATA'][id]['STATUS'] == 'Error':
                                temp_thread.temp_control_start = False
                                shared_memory_u[0x18 + id*self.shared_memory_size] = int(command['DATA'][id]['STAGE']) << 16 | 0
                                status = 5
                            else:
                                status = 10
                            shared_memory_u[0x18 + id*self.shared_memory_size] = int(command['DATA'][id]['STAGE']) << 16 | status
                            
                            if command['DATA'][id]['STATUS'] == 'Run' and int(command['DATA'][id]['STAGE']) != old_stage:
                                shared_memory_u[0x17 + id*self.shared_memory_size] = 0
                            self.old_stage = int(command['DATA'][id]['STAGE'])
                    elif command['CMD'] == 'SET_MOTOR':
                        if int(self.config['ADDRESS'], 16) != 0xFFF:
                            message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+id,  
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
                                
                            while not can_fd_receive_queue.empty():
                                can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                            
                            self.can_fd_transmitte_queue.put(message) 
                            # time.sleep(0.40)
                            wait = 0
                            while can_fd_receive_queue.empty():
                                time.sleep(0.01)
                                wait += 1
                                if wait > 120:
                                    break
                            if not can_fd_receive_queue.empty():
                                message = can_fd_receive_queue.get()
                                if message.data[1] == 0x11:
                                    logging.info(f'id : {id} Received message: {message}')
                                    if self.socket[0]:
                                        if message.data[4] == 1:
                                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":f"success!"}), 'UTF-8'))
                                        else:
                                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":"fail!"}), 'UTF-8'))
                                else:
                                    logging.warning(f'id : {id} {command["CMD"]} unit board is wrong response')  
                            else:
                                logging.warning(f'id : {id} {command["CMD"]} unit board is not response')   
                            # logging.info(f'id : {id} UnitBoard execute {command["CMD"]}')            
                    elif command['CMD'] == 'GET_ADC':
                        if int(self.config['ADDRESS'], 16) != 0xFFF:
                            message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+id,  
                                        data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
                            
                            while not can_fd_receive_queue.empty():
                                can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                
                            self.can_fd_transmitte_queue.put(message) 
                            # time.sleep(0.06)
                            wait = 0
                            while can_fd_receive_queue.empty():
                                time.sleep(0.01)
                                wait += 1
                                if wait > 120:
                                    break
                            if not can_fd_receive_queue.empty():
                                message = can_fd_receive_queue.get()
                                if message.data[1] == 0x12:
                                    logging.info(f'id : {id} Received message: {message}')
                                    unit_semaphor.acquire()
                                    shared_memory_u[0x00 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[4] << 8) | (np.int32)(message.data[5]))
                                    shared_memory_u[0x01 + id*self.shared_memory_size]  = (np.int32)((np.int32)(message.data[6] << 8) | (np.int32)(message.data[7]))
                                    shared_memory_u[0x02 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[8] << 8) | (np.int32)(message.data[9]))
                                    shared_memory_u[0x03 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[10] << 8) | (np.int32)(message.data[11]))
                                    shared_memory_u[0x04 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[12] << 8) | (np.int32)(message.data[13]))
                                    shared_memory_u[0x05 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[14] << 8) | (np.int32)(message.data[15]))
                                    unit_semaphor.release()
                                    if self.socket[0]:
                                        self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":"success!", 
                                                    "ADC0": f'{shared_memory_u[0x00 + id*self.shared_memory_size]}',
                                                    "ADC1": f'{shared_memory_u[0x01 + id*self.shared_memory_size]}',
                                                    "ADC2": f'{shared_memory_u[0x02 + id*self.shared_memory_size]}',
                                                    "ADC3": f'{shared_memory_u[0x03 + id*self.shared_memory_size]}',
                                                    "ADC4": f'{shared_memory_u[0x04 + id*self.shared_memory_size]}',
                                                    "ADC5": f'{shared_memory_u[0x05 + id*self.shared_memory_size]}'}), 'UTF-8'))
                                else:
                                    logging.warning(f'id : {id} {command["CMD"]} unit board is wrong response') 
                            else:
                                logging.warning(f'id : {id} {command["CMD"]} unit board is not response') 
                            # logging.info(f'id : {id} UnitBoard execute {command["CMD"]}')
                    elif command['CMD'] == 'SET_GPIO':
                        if int(self.config['ADDRESS'], 16) != 0xFFF:
                            message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+id,  
                                        data=[0xF2, 0x13, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                            message.data[4] = len(command['NUM'])
                            for i in range(message.data[4]):
                                message.data[5 + i] = command['VALUE'][i]
                            
                            while not can_fd_receive_queue.empty():
                                can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                
                            self.can_fd_transmitte_queue.put(message) 
                            wait = 0
                            while can_fd_receive_queue.empty():
                                time.sleep(0.01)
                                wait += 1
                                if wait > 120:
                                    break
                            
                            if not can_fd_receive_queue.empty():
                                message = can_fd_receive_queue.get()
                                if message.data[1] == 0x13:
                                    logging.info(f'id : {id} Received message: {message}')
                                    if self.socket[0]:
                                        self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":"success!"}), 'UTF-8'))
                                else:
                                    logging.warning(f'id : {id} {command["CMD"]} unit board is wrong response') 
                            else:
                                logging.warning(f'id : {id} {command["CMD"]} unit board is not response') 
                            # logging.info(f'id : {id} UnitBoard execute {command["CMD"]}')
                    elif command['CMD'] == 'GET_STATUS':
                        if int(self.config['ADDRESS'], 16) != 0xFFF:
                            # 온도계산 전에 GET_ADC를 호출 함. --> 명령어를 통합하여 ADC 값까지 GET_STATUS에서 읽어 옴.
                            message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+id,  
                                        data=[0xF2, 0x14, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                            
                            while not can_fd_receive_queue.empty():
                                can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                            
                            self.can_fd_transmitte_queue.put(message) 
                            wait = 0
                            while can_fd_receive_queue.empty():
                                time.sleep(0.01)
                                wait += 1
                                if wait > 120:
                                    break
                            
                            if not can_fd_receive_queue.empty():
                                message = can_fd_receive_queue.get()
                                if message.data[1] == 0x14:
                                    if command['SEND']:                      # 로고에 너무 많이 쌓이는 데이터 방지, 
                                        logging.info(f'id : {id} Received message: {message}')
                                    inclination1 = 77.5 / (float(self.config['TEMP1_77_5']) - float(self.config['TEMP1_0']))
                                    y_offset1 = inclination1 * float(self.config['TEMP1_0'])
                                    
                                    inclination2 = 77.5 / (float(self.config['TEMP2_77_5']) - float(self.config['TEMP2_0']))
                                    y_offset2 = inclination2 * float(self.config['TEMP2_0'])
                                    
                                    # inclination3 = 77.5 / (float(self.config['TEMP3_77_5']) - float(self.config['TEMP3_0']))
                                    # y_offset3 = inclination3 * float(self.config['TEMP3_0'])
                                    
                                    # inclination4 = 77.5 / (float(self.config['TEMP4_77_5']) - float(self.config['TEMP4_0']))
                                    # y_offset4 = inclination4 * float(self.config['TEMP4_0'])
                                        
                                    unit_semaphor.acquire()
                                    shared_memory_u[0x06 + id*self.shared_memory_size] = (np.int32)(message.data[7] << 24 | message.data[6] << 16 
                                                                    | message.data[5] << 8 | message.data[4])
                                    shared_memory_u[0x07 + id*self.shared_memory_size] = (np.int32)(message.data[9] << 8 | message.data[8])
                                    shared_memory_u[0x08 + id*self.shared_memory_size] = (np.int32)(message.data[13] << 24 | message.data[12] << 16 
                                                                    | message.data[11] << 8 | message.data[10])
                                    shared_memory_u[0x09 + id*self.shared_memory_size] = (np.int32)(message.data[14])
                                    shared_memory_u[0x0A + id*self.shared_memory_size] = (np.int32)(message.data[15] << 8 | message.data[16]) #RPM
                                    shared_memory_u[0x0B + id*self.shared_memory_size] = (np.int32)(message.data[17] << 8 | message.data[18]) #load cell
                                    
                                    shared_memory_u[0x0C + id*self.shared_memory_size] = (np.int32)(message.data[19] << 8 | message.data[20]) #Ext Temp1
                                    shared_memory_u[0x0D + id*self.shared_memory_size] = (np.int32)(message.data[21] << 8 | message.data[22]) #Ext Humi1
                                    shared_memory_u[0x0E + id*self.shared_memory_size] = (np.int32)(message.data[23] << 8 | message.data[24]) #Ext Temp2
                                    shared_memory_u[0x0F + id*self.shared_memory_size] = (np.int32)(message.data[25] << 8 | message.data[26]) #Ext Humi2
                                    
                                    # ADC 값
                                    shared_memory_u[0x00 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[27] << 8) | (np.int32)(message.data[28]))
                                    shared_memory_u[0x01 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[29] << 8) | (np.int32)(message.data[30]))
                                    shared_memory_u[0x02 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[31] << 8) | (np.int32)(message.data[32]))
                                    shared_memory_u[0x03 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[33] << 8) | (np.int32)(message.data[34]))
                                    shared_memory_u[0x04 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[35] << 8) | (np.int32)(message.data[36]))
                                    shared_memory_u[0x05 + id*self.shared_memory_size] = (np.int32)((np.int32)(message.data[37] << 8) | (np.int32)(message.data[38]))

                                    # ADC 값에 온도 계산식을 추가해서 공유메모리에 저장 여기서는 2개만 계산하고 필요하면 추가.
                                    shared_memory_u[0x10 + id*self.shared_memory_size] = float(f'{(inclination1 * shared_memory_u[0 + id*self.shared_memory_size] - y_offset1) * 100 : 0.2F}')
                                    shared_memory_u[0x11 + id*self.shared_memory_size] = float(f'{(inclination2 * shared_memory_u[1 + id*self.shared_memory_size] - y_offset2) * 100 : 0.2F}')
                                    unit_semaphor.release()
                                    
                                    # print(f'top temp. is {shared_memory_u[0x10 + id*self.shared_memory_size]*0.01:0.2F} and bottom temp. is {shared_memory_u[0x11 + id*self.shared_memory_size]*0.01}')
                                    # shared_memory_u[0x12 + id*self.shared_memory_size] = float(f'{(inclination3 * shared_memory_u[2 + id*self.shared_memory_size] - y_offset3) * 100 : 0.2F}')
                                    # shared_memory_u[0x13 + id*self.shared_memory_size] = float(f'{(inclination4 * shared_memory_u[3 + id*self.shared_memory_size] - y_offset4) * 100 : 0.2F}')
                                        
                                    if command['SEND'] and self.socket[0]:
                                        self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":"success!", 
                                            "TEMP1" : f'{shared_memory_u[0x10 + id*self.shared_memory_size] * 0.01: 0.2F}',
                                            "TEMP2" : f'{shared_memory_u[0x11 + id*self.shared_memory_size] * 0.01: 0.2F}',
                                            "GPO4~GPO1": f'{shared_memory_u[0x06 + id*self.shared_memory_size]}',
                                            "GPO8~GPO5": f'{shared_memory_u[0x07 + id*self.shared_memory_size]}',
                                            "GPI4~GPI1": f'{shared_memory_u[0x08 + id*self.shared_memory_size]}',
                                            "GPI8~GPI5": f'{shared_memory_u[0x09 + id*self.shared_memory_size]}',
                                            "RPM": f'{shared_memory_u[0x0A + id*self.shared_memory_size]}',
                                            "LOAD CELL": f'{shared_memory_u[0x0B + id*self.shared_memory_size] * 0.01 : 0.2F}kg',
                                            "EXTTEMP1": f'{shared_memory_u[0x0C + id*self.shared_memory_size] * 0.01 : 0.2F}C',
                                            "EXTHUMI1": f'{shared_memory_u[0x0D + id*self.shared_memory_size] * 0.01 : 0.2F}%',
                                            "EXTTEMP2": f'{shared_memory_u[0x0E + id*self.shared_memory_size] * 0.01 : 0.2F}C',
                                            "EXTHUMI2": f'{shared_memory_u[0x0F + id*self.shared_memory_size] * 0.01 : 0.2F}%',
                                            }), 'UTF-8'))
                                else:
                                    logging.warning(f'id : {id} {command["CMD"]} unit board is wrong response')
                            else:
                                logging.warning(f'id : {id} {command["CMD"]} unit board is not response') 
                            # logging.info(f'id : {id} UnitBoard execute {command["CMD"]}')
                    elif command['CMD'] == 'START_TEMP':
                        if int(self.config['ADDRESS'], 16) != 0xFFF:
                            event.set()
                            temp_thread.temp_control_start = True
                            
                            if self.socket[0]:
                                self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":"success!"}), 'UTF-8'))
                            # logging.info(f'id : {id} UnitBoard execute {command["CMD"]}')
                    elif command['CMD'] == 'STOP_TEMP':
                        if int(self.config['ADDRESS'], 16) != 0xFFF:
                            temp_thread.temp_control_start = False
                            
                            if self.socket[0]:
                                self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":"success!"}), 'UTF-8'))
                            # logging.info(f'id : {id} UnitBoard execute {command["CMD"]}')
                    elif command['CMD'] == 'TEMP_RPM':
                        if int(self.config['ADDRESS'], 16) != 0xFFF:
                            message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+id,  
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
                                
                            while not can_fd_receive_queue.empty():
                                can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                            
                            self.can_fd_transmitte_queue.put(message) 
                            wait = 0
                            while can_fd_receive_queue.empty():
                                time.sleep(0.01)
                                wait += 1
                                if wait > 120:
                                    break
                            
                            if not can_fd_receive_queue.empty():
                                message = can_fd_receive_queue.get()
                                if message.data[1] == 0x17:
                                    logging.info(f'id : {id} Received message: {message}')
                                    if command['SEND'] and self.socket[0]:
                                        if message.data[4] == 1:
                                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":f"success!"}), 'UTF-8'))
                                        else:
                                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":"fail!"}), 'UTF-8'))
                                else:
                                    logging.warning(f'id : {id} {command["CMD"]} unit board is wrong response')  
                            else:
                                logging.warning(f'id : {id} {command["CMD"]} unit board is not response')   
                            # logging.info(f'id : {id} UnitBoard execute {command["CMD"]}')
                    elif command['CMD'] == 'TEMP_VALVE':
                        if int(self.config['ADDRESS'], 16) != 0xFFF:
                            message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+id,  
                                        data=[0xF2, 0x18, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])

                            message.data[4] = int(command['CHANNEL'])             
                            message.data[5] = command['VALUE']          # 유닛보드에서 설정된 포트 값
                                
                            while not can_fd_receive_queue.empty():
                                can_fd_receive_queue.get()              # as docs say: Remove and return an item from the queue.
                            
                            self.can_fd_transmitte_queue.put(message) 
                            wait = 0
                            while can_fd_receive_queue.empty():
                                time.sleep(0.01)
                                wait += 1
                                if wait > 120:
                                    break
                            
                            if not can_fd_receive_queue.empty():
                                message = can_fd_receive_queue.get()
                                if message.data[1] == 0x18:
                                    logging.info(f'id : {id} Received message: {message}')
                                    # temp_thread.set_cold_valve(message.data[5])   # 지속적인 재전송을 원하면 주석 해제. 
                                    
                                    if self.socket[0]:
                                        if message.data[4] == 1:
                                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":f"success!"}), 'UTF-8'))
                                        else:
                                            self.socket[0].sendall(bytes(json.dumps({"id" : f'{id}', "status":"fail!"}), 'UTF-8'))
                                else:
                                    logging.warning(f'id : {id} {command["CMD"]} unit board is wrong response')  
                            else:
                                logging.warning(f'id : {id} {command["CMD"]} unit board is not response')  
                            # logging.info(f'id : {id} UnitBoard execute {command["CMD"]}')      
                    elif command['CMD'] == 'CTRL':
                        if int(self.config['TANK_ID']) == int(command['TANK_ID']) and int(self.config['ADDRESS'], 16) != 0xFFF:
                            if command['CTRL'][0]['SENSOR_ID'] == '500':    #밸브는 4개 밸브 아이디는 500부터 시작 500-> 냉각
                                x = self.config["SOLVALVE2"]                #밸브 I/O 번호
                                if command['CTRL'][0]['PARAM0'] == 'ON':
                                    value = ON
                                else:
                                    value = OFF
                                message = {"UNIT_ID" : id,                  
                                            "CMD":"TEMP_VALVE",
                                            "CHANNEL": x,
                                            "VALUE" : value}
                                command_queue.put(message) 
                            elif command['CTRL'][0]['SENSOR_ID'] == '501':  #밸브는 4개 밸브 아이디는 500부터 시작 501-> 워터
                                x = self.config["SOLVALVE1"]                #밸브 I/O 번호
                                if command['CTRL'][0]['PARAM0'] == 'ON':
                                    value = ON
                                else:
                                    value = OFF
                                #
                                if int(self.config['ADDRESS'], 16) != 0xFFF:
                                    message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+id,  
                                                data=[0xF2, 0x19, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                                
                                temp = int(float(command['CTRL'][0]['PARAM1'])) * 10
                                message.data[4] = int(x)
                                message.data[5] = value
                                message.data[7] = temp & 0xff               # big endian
                                message.data[6] = (temp >> 8) & 0xff        # big endian
                                message.data[8] = int(self.config["WATER_VALVE_ON_TIME"])
                                    
                                while not can_fd_receive_queue.empty():
                                    can_fd_receive_queue.get()             # as docs say: Remove and return an item from the queue.
                                
                                self.can_fd_transmitte_queue.put(message) 
                                # time.sleep(0.40)
                                wait = 0
                                while can_fd_receive_queue.empty():
                                    time.sleep(0.01)
                                    wait += 1
                                    if wait > 120:
                                        break
                                if not can_fd_receive_queue.empty():
                                    message = can_fd_receive_queue.get()
                                    if message.data[1] == 0x19:
                                        logging.info(f'id : {id} Received message: {message}')
                                    else:
                                        logging.warning(f'id : {id} {command["CMD"]} unit board is wrong response')  
                                else:
                                    logging.warning(f'id : {id} {command["CMD"]} unit board is not response')   
                            elif command['CTRL'][0]['SENSOR_ID'] == '502':
                                pass
                            elif command['CTRL'][0]['SENSOR_ID'] == '503':
                                pass
                            elif command['CTRL'][0]['SENSOR_ID'] == '600':     #모터 1개 모터 아이디는 600부터 시작
                                rpm = int(command['CTRL'][0]['PARAM0'])
                                run_time = int(command['CTRL'][0]['PARAM1'])
                                message = {"UNIT_ID" : id,                  
                                        "CMD":"TEMP_RPM",
                                        "SPEED" : rpm, 
                                        "DIR"   : 'FW',            #FW = forward, RV = reverse
                                        "ONOFF" : 'ON', 
                                        "TIME" : run_time,
                                        "SEND" : False}    
                                command_queue.put(message)
                self.i2c_semaphor.acquire()
                i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF)
                i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF)
                i2cbus.close()
                self.i2c_semaphor.release()
            except Exception as e:
                print(e)
                i2cbus.close()
                self.i2c_semaphor.release()
                unit_semaphor.release()
                logging.error(f'id : {id} {e}')
                print(traceback.print_exc())
