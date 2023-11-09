#! /usr/bin/python3
import os
import can
import time

from concurrent.futures import ThreadPoolExecutor
import threading

class CanFDReceive(threading.Thread):
    def __init__(self, logging, main_func) -> None:
        threading.Thread.__init__(self) 
        self.can0 = main_func.can0
        self.logging = logging
        self.daemon = True
        self.receive_queue = []
        self.logging.info('CanFDReceive initialized')
        
    def run(self): 
        while True:
            message = self.can0.recv()
            # self.can0.shutdown()
            if message.arbitration_id >= 0x300 and message.arbitration_id <= 0x31F:
                self.receive_queue[message.arbitration_id - 0x300].put(message)
            else:
                self.logging.warning(f'Wrong arbitration_id {message.arbitration_id}')

class CanFDTransmitte(threading.Thread):
    def __init__(self, logging, main_func) -> None:
        threading.Thread.__init__(self) 
        self.can0 = main_func.can0
        self.logging = logging
        self.logging.info('CanFDTransmitte initialized')
        self.daemon = True
        self.queue = None
        self.parsing = {
            0x11 : 'MOTOR_SPEED',
            0x12 : 'GET_ADC',
            0x13 : 'SET_GPIO',
            0x14 : 'GET_STATUS',
            0x15 : 'SET_TEMP',
            0x16 : 'SET_CONFIG',
            0x17 : 'TEMP_RPM',
            0x18 : 'TEMP_VALVE',
            0x19 : 'WEIGHT_VALVE',
            0x21 : 'SET_LED'
        }
    def run(self): 
        while True:
            message = self.queue.get()
            if message.data[1] != 0x14: # STATUS 명령어는 표시 하지 않음.
                self.logging.info(f"CAN {self.parsing[message.data[1]]} command is inserted Unit Board")
            self.can0.send(message)
