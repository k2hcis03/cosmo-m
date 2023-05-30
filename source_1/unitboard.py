#! /usr/bin/python3

import smbus
import os
import can
import time
import numpy as np
import json
from multiprocessing import Process, Queue, Manager, shared_memory

class UnitBoard:
    def __init__(self, logging, transmitte_queue, shared_socket) -> None:
        self.transmitte_queue = transmitte_queue
        self.receive_queue = None
        self.command_queue = None
        self.logging = logging
        self.shared_memory = None
        self.unit_semaphor = None
        self.shared_socket = shared_socket
        
    def unit_process(self, n, shm, arr, semaphor, receive_queue, command_queue, config):
        new_shm = shared_memory.SharedMemory(name=shm)
        self.shared_memory = np.ndarray(arr.shape, dtype=arr.dtype, buffer=new_shm.buf)
        self.logging.info(f'Process {os.getpid()} and {n} are created')  
        self.id = n
        self.unit_semaphor = semaphor
        self.receive_queue = receive_queue
        self.command_queue = command_queue
        self.config = config
        
        while True:
            command = self.command_queue.get()
            if not command:
                self.logging.warning('Timeout waiting for command')
            else: 
                if command['cmd'] == 'SET_MOTOR':
                    message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                  data=[0xF2, 0x11, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF3])
                    
                    temp = int(command['speed'])
                    message.data[5] = temp & 0xff               #big endian
                    message.data[4] = (temp >> 8) & 0xff        #big endian
                    dir = int(command['dir'])
                    message.data[6] = dir
                    
                    if (command['onoff'] == 'ON'):
                        message.data[7] = 1
                    else:
                        message.data[7] = 0
                        
                    self.transmitte_queue.put(message) 
                    
                    time.sleep(0.1)
                    if not self.receive_queue.empty():
                        message = self.receive_queue.get()
                        self.logging.info(f'Received message: {message}')
                        
                        if self.shared_socket[0]:
                            print(self.shared_socket[0])
                            if message.data[4] == 1:
                                self.shared_socket[0].sendall(bytes(json.dumps({"status":"success!"}), 'UTF-8'))
                            else:
                                self.shared_socket[0].sendall(bytes(json.dumps({"status":"fail!"}), 'UTF-8'))
                    else:
                        self.logging.warning('unit board is not response')               
                elif command['cmd'] == 'GET_ADC':
                    message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                  data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
                    self.transmitte_queue.put(message) 
                    
                    time.sleep(0.1)
                    if not self.receive_queue.empty():
                        message = self.receive_queue.get()
                        self.logging.info(f'Received message: {message}')
                        self.unit_semaphor.acquire()
                        self.shared_memory[0] = (np.int32)((np.int32)(message.data[5] << 8) | (np.int32)(message.data[4]))
                        self.shared_memory[1] = (np.int32)((np.int32)(message.data[7] << 8) | (np.int32)(message.data[6]))
                        self.shared_memory[2] = (np.int32)((np.int32)(message.data[9] << 8) | (np.int32)(message.data[8]))
                        self.shared_memory[3] = (np.int32)((np.int32)(message.data[11] << 8) | (np.int32)(message.data[10]))
                        self.shared_memory[4] = (np.int32)((np.int32)(message.data[13] << 8) | (np.int32)(message.data[12]))
                        self.shared_memory[5] = (np.int32)((np.int32)(message.data[15] << 8) | (np.int32)(message.data[14]))
                        self.unit_semaphor.release()
                        
                        if self.shared_socket[0]:
                            self.shared_socket[0].sendall(bytes(json.dumps({"status":"success!", 
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
                    self.transmitte_queue.put(message) 
                    
                    time.sleep(0.1)
                    if not self.receive_queue.empty():
                        message = self.receive_queue.get()
                        self.logging.info(f'Received message: {message}')
                        
                        if self.shared_socket[0]:
                            self.shared_socket[0].sendall(bytes(json.dumps({"status":"success!"}), 'UTF-8'))
                    else:
                        self.logging.warning('unit board is not response')
                elif command['cmd'] == 'GET_STATUS':
                    # 온도계산 전에 GET_ADC를 호출 함.
                    message = can.Message(is_extended_id=False, is_fd = True, arbitration_id=0x300+self.id,  
                                  data=[0xF2, 0x12, 0x05, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF3])
                    self.transmitte_queue.put(message) 
                    
                    time.sleep(0.1)
                    if not self.receive_queue.empty():
                        message = self.receive_queue.get()
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
                    self.transmitte_queue.put(message) 
                    
                    time.sleep(0.1)
                    if not self.receive_queue.empty():
                        
                        message = self.receive_queue.get()
                        self.logging.info(f'Received message: {message}')
                        
                        self.unit_semaphor.acquire()
                        self.shared_memory[6] = (np.int32)(message.data[7] << 24 | message.data[6] << 16 
                                                           | message.data[5] << 8 | message.data[4])
                        self.shared_memory[7] = (np.int32)(message.data[9] << 8 | message.data[8])
                        self.shared_memory[8] = (np.int32)(message.data[13] << 24 | message.data[12] << 16 
                                                           | message.data[11] << 8 | message.data[10])
                        self.shared_memory[9] = (np.int32)(message.data[15] << 8 | message.data[14])
                        self.unit_semaphor.release()
                        
                        if self.shared_socket[0]:
                            inclination = 77.5 / (float(self.config['TEMP1_77_5']) - float(self.config['TEMP1_0']))
                            y_offset = inclination * float(self.config['TEMP1_0'])
                            self.shared_socket[0].sendall(bytes(json.dumps({"status":"success!", 
                                                                  "TEMP1" : f'{inclination * self.shared_memory[0] - y_offset}',
                                                                  "GPO4~GPO1": f'{self.shared_memory[6]}',
                                                                  "GPO8~GPO5": f'{self.shared_memory[7]}',
                                                                  "GPI4~GPI1": f'{self.shared_memory[8]}',
                                                                  "GPI8~GPI5": f'{self.shared_memory[9]}',
                                                                  }), 'UTF-8'))
                    else:
                        self.logging.warning('unit board is not response')
            
            
           