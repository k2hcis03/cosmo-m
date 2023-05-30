#! /usr/bin/python3
import os
import can
import time

from concurrent.futures import ThreadPoolExecutor
import threading
import time

address = 0x20

class CanFDReceive(threading.Thread):
    def __init__(self, logging, main_func) -> None:
        threading.Thread.__init__(self) 
        self.can0 = main_func.can0
        self.logging = logging
        self.logging.info('CanFDReceive initialized')
        self.daemon = True

    def run(self): 
        while True:
            time.sleep(0.1)
            msg = self.can0.recv()
            print(msg)
        # msg = can.Message(is_extended_id=False, arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7])
        # can0.send(msg)
        # time.sleep(0.1)
        # msg = can0.recv(10.0)
        # os.system('sudo ifconfig can0 down')
        # message = bus.recv()
		# if message.arbitration_id == PID_REPLY:
		# 	q.put(message)			# Put message into queue

class CanFDTransmitte(threading.Thread):
    def __init__(self, logging, main_func) -> None:
        threading.Thread.__init__(self) 
        self.can0 = main_func.can0
        self.logging = logging
        self.logging.info('CanFDTransmitte initialized')
        self.daemon = True
        self.queue = main_func.transmitte_queue
        
    def run(self): 
        while True:
            msg = self.queue.get()
            # msg = can.Message(is_extended_id=False, arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7])
            # msg.is_rx = False
            self.can0.send(msg)
            self.logging.info(f'CanFD Message is sent {msg.data}')
