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

class UnitBoardTempControl(threading.Thread):
    def __init__(self, id, queue, lock, event, logging, can_fd_transmitte_queue, can_fd_receive_queue, command_queue):
        threading.Thread.__init__(self)
        self.daemon = True
        self.id = id + 1            # id는 0부터 시작하므로 1을 더해줌.
        self.logging = logging
        self.can_fd_transmitte_queue = can_fd_transmitte_queue
        self.can_fd_receive_queue = can_fd_receive_queue
        self.lock = lock
        self.event = event
        self.queue = queue
        self.command_queue = command_queue
    
    # def can_fd_response(self):
    #     if not self.can_fd_receive_queue.empty():
    #         message = self.can_fd_receive_queue.get()
    #         self.logging.info(message)
    #     else:
    #         self.logging.warning('unit board is not response')
            
    def run(self):
            self.logging.info('UnitBoard Temp Control Thread Run')
            while True:
                try:
                    self.event.wait()
                    data = self.queue.get()
                    mode = data['mode']
                    temp = data['temp']
                    time_out = data['time_out']
                    
                    message = {"unit_id" : self.id,
                        "cmd":"SET_GPIO",
                        "num" : [0, 1, 2, 3], 
                        "value" : [False, False, False, False]}
                    
                    self.command_queue.put(message)
                except Exception as e:
                    print(e)

class UnitBoard:
    def __init__(self, logging, transmitte_queue, shared_object, GPIOADDR, i2c_semaphor) -> None:
        self.can_fd_transmitte_queue = transmitte_queue
        self.logging = logging
        self.socket = shared_object
        self.GPIOADDR = GPIOADDR
        self.i2c_semaphor = i2c_semaphor
        
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
        
        temp_thread = UnitBoardTempControl(self.id, self.queue, self.lock, self.event, self.logging, self.can_fd_transmitte_queue, 
                                                           self.can_fd_receive_queue, self.command_queue)
        temp_thread.start()
                        
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
                    self.logging.warning('Timeout waiting for command')
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
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.10)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'Received message: {message}')
                            if self.socket[0]:
                                if message.data[4] == 1:
                                    self.socket[0].sendall(bytes(json.dumps({"status":"success!"}), 'UTF-8'))
                                else:
                                    self.socket[0].sendall(bytes(json.dumps({"status":"fail!"}), 'UTF-8'))
                        else:
                            self.logging.warning('unit board is not response')               
                    elif command['cmd'] == 'GET_ADC':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.05)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'Received message: {message}')
                            self.unit_semaphor.acquire()
                            self.shared_memory[0] = (np.int32)((np.int32)(message.data[5] << 8) | (np.int32)(message.data[4]))
                            self.shared_memory[1] = (np.int32)((np.int32)(message.data[7] << 8) | (np.int32)(message.data[6]))
                            self.shared_memory[2] = (np.int32)((np.int32)(message.data[9] << 8) | (np.int32)(message.data[8]))
                            self.shared_memory[3] = (np.int32)((np.int32)(message.data[11] << 8) | (np.int32)(message.data[10]))
                            self.shared_memory[4] = (np.int32)((np.int32)(message.data[13] << 8) | (np.int32)(message.data[12]))
                            self.shared_memory[5] = (np.int32)((np.int32)(message.data[15] << 8) | (np.int32)(message.data[14]))
                            self.unit_semaphor.release()
                            if self.socket[0]:
                                self.socket[0].sendall(bytes(json.dumps({"status":"success!", 
                                                                    "ADC0": f'{self.shared_memory[0]}',
                                                                    "ADC1": f'{self.shared_memory[1]}',
                                                                    "ADC2": f'{self.shared_memory[2]}',
                                                                    "ADC3": f'{self.shared_memory[3]}',
                                                                    "ADC4": f'{self.shared_memory[4]}',
                                                                    "ADC5": f'{self.shared_memory[5]}'}), 'UTF-8'))
                        else:
                            self.logging.warning('unit board is not response')
                    elif command['cmd'] == 'SET_GPIO':
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x13, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        message.data[4] = len(command['num'])
                        for i in range(message.data[4]):
                            message.data[5 + i] = command['value'][i]
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.05)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'Received message: {message}')
                            if self.socket[0]:
                                self.socket[0].sendall(bytes(json.dumps({"status":"success!"}), 'UTF-8'))
                        else:
                            self.logging.warning('unit board is not response')
                    elif command['cmd'] == 'GET_STATUS':
                        # 온도계산 전에 GET_ADC를 호출 함.
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.05)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'Received message: {message}')
                            self.unit_semaphor.acquire()
                            self.shared_memory[0] = (np.int32)((np.int32)(message.data[5] << 8) | (np.int32)(message.data[4]))
                            self.shared_memory[1] = (np.int32)((np.int32)(message.data[7] << 8) | (np.int32)(message.data[6]))
                            self.shared_memory[2] = (np.int32)((np.int32)(message.data[9] << 8) | (np.int32)(message.data[8]))
                            self.shared_memory[3] = (np.int32)((np.int32)(message.data[11] << 8) | (np.int32)(message.data[10]))
                            self.shared_memory[4] = (np.int32)((np.int32)(message.data[13] << 8) | (np.int32)(message.data[12]))
                            self.shared_memory[5] = (np.int32)((np.int32)(message.data[15] << 8) | (np.int32)(message.data[14]))
                            self.unit_semaphor.release()
                        else:
                            self.logging.warning('unit board is not response')
                        # GET_ADC후에 GET_STATUS 수행
                        message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                    data=[0xF2, 0x14, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                        self.can_fd_transmitte_queue.put(message) 
                        time.sleep(0.05)
                        
                        if not self.can_fd_receive_queue.empty():
                            message = self.can_fd_receive_queue.get()
                            self.logging.info(f'Received message: {message}')
                            self.unit_semaphor.acquire()
                            self.shared_memory[6] = (np.int32)(message.data[7] << 24 | message.data[6] << 16 
                                                            | message.data[5] << 8 | message.data[4])
                            self.shared_memory[7] = (np.int32)(message.data[9] << 8 | message.data[8])
                            self.shared_memory[8] = (np.int32)(message.data[13] << 24 | message.data[12] << 16 
                                                            | message.data[11] << 8 | message.data[10])
                            self.shared_memory[9] = (np.int32)(message.data[14])
                            self.shared_memory[10] = (np.int32)(message.data[15] << 8 | message.data[16]) #RPM
                            self.shared_memory[11] = (np.int32)(message.data[17] << 8 | message.data[18]) #load cell
                            self.unit_semaphor.release()
                            if self.socket[0]:
                                inclination1 = 77.5 / (float(self.config['TEMP1_77_5']) - float(self.config['TEMP1_0']))
                                y_offset1 = inclination1 * float(self.config['TEMP1_0'])
                                
                                inclination2 = 77.5 / (float(self.config['TEMP2_77_5']) - float(self.config['TEMP2_0']))
                                y_offset2 = inclination2 * float(self.config['TEMP2_0'])
                                self.socket[0].sendall(bytes(json.dumps({"status":"success!", 
                                                                    "TEMP1" : f'{inclination1 * self.shared_memory[0] - y_offset1}',
                                                                    "TEMP2" : f'{inclination2 * self.shared_memory[1] - y_offset2}',
                                                                    "GPO4~GPO1": f'{self.shared_memory[6]}',
                                                                    "GPO8~GPO5": f'{self.shared_memory[7]}',
                                                                    "GPI4~GPI1": f'{self.shared_memory[8]}',
                                                                    "GPI8~GPI5": f'{self.shared_memory[9]}',
                                                                    "RPM": f'{self.shared_memory[10]}',
                                                                    "LOAD CELL": f'{self.shared_memory[11] * 0.01 : 0.2F}kg',
                                                                    }), 'UTF-8'))
                        else:
                            self.logging.warning('unit board is not response')
                    elif command['cmd'] == 'SET_TEMP':
                        temp = int(command['temp'])
                        if command['mode'] == 'BOTH':
                            mode = 0
                        elif command['mode'] == 'MOTOR':
                            mode = 1
                        else:
                            mode = 2
                        time_out = int(command['time_out'])
                        
                        self.queue.put({
                            'temp' : temp,
                            'mode' : mode,
                            'time_out' : time_out
                        })
                        self.event.set()
                        # 여기서는 따로 응답하지 않고 위에서 정의 된 명령어에서 응답함.
                            
                i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF)
                i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF)
                i2cbus.close()
                self.i2c_semaphor.release()
            except Exception as e:
                print(e)
                self.logging.error(f'{e}')
           