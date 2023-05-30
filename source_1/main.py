#! /usr/bin/python3

import smbus
import os
import can
import time

from logger import logger as logging
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing import Process, current_process, Queue, Manager, shared_memory
import threading
import time
import canfd
import queue
from concurrent.futures import wait
import numpy as np
from konfig import Config

from unitboard import UnitBoard as unit_board
from client import TcpClientThread as tcp_client

shared_socket = Manager().list()            # 프로세스간 공유 소켓 정보
shared_socket.insert(0, None)

GPIOADDR = 0x20
MAXUNITBOARD = 1
         
class CosmoMain(threading.Thread):
    def __init__(self, tcp_queue) -> None:
        threading.Thread.__init__(self) 
        os.system("sudo ip link set can0 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on")
        os.system("sudo ifconfig can0 txqueuelen 65536")
        
        self.client = None
        self.tcp_queue = tcp_queue
        
        filters = [
            {"can_id": 0x300, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x301, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x302, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x303, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x304, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x305, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x306, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x307, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x308, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x309, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x30A, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x30B, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x30C, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x30D, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x30E, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x30F, "can_mask": 0x7FF, "extended": False},
        ]
        try:
            self.can0 = can.interface.Bus(rx_fifo_size = 8192, channel = 'can0', bustype = 'socketcan', bitrate_switch = True, bitrate = 1000000, data_bitrate = 8000000, fd = True, can_filters=filters)  # socketcan_native
        except Exception as e:
            print(e)
            
        self.i2cbus = smbus.SMBus(1) 
        self.i2cbus.write_byte_data(GPIOADDR, 0x00, 0x00)        # OUTPUT
        self.i2cbus.write_byte_data(GPIOADDR, 0x01, 0x00)        # OUTPUT
        self.i2cbus.write_byte_data(GPIOADDR, 0x12, 0xFF)
        self.i2cbus.write_byte_data(GPIOADDR, 0x13, 0xFF)
        self.command_queue = []
        
        self.unit_np_shm = None
        self.unit_semaphor = None
        
    def run(self):
        while True:   
            message = self.tcp_queue.get()   
            logging.info(f"{message['cmd']} command is inserted to {message['unit_id']} Unit Board")
            
            if message['cmd'] == 'SET_MOTOR':
                if message['unit_id'] > 0 and message['unit_id'] <= MAXUNITBOARD:
                    self.command_queue[message['unit_id'] - 1].put(message)
                else:
                    logging.info(f"Wrong Unit board id{message['unit_id'] - 1}")
            elif message['cmd'] == 'SET_GPIO':
                if message['unit_id'] > 0 and message['unit_id'] <= MAXUNITBOARD:
                    self.command_queue[message['unit_id'] - 1].put(message)
                else:
                    logging.info(f"Wrong Unit board id{message['unit_id'] - 1}")
            elif message['cmd'] == 'GET_ADC':  
                if message['unit_id'] > 0 and message['unit_id'] <= MAXUNITBOARD:
                    self.command_queue[message['unit_id'] - 1].put(message)
                else:
                    logging.info(f"Wrong Unit board id{message['unit_id'] - 1}")
            elif message['cmd'] == 'GET_STATUS':  
                if message['unit_id'] > 0 and message['unit_id'] <= MAXUNITBOARD:
                    self.command_queue[message['unit_id'] - 1].put(message)
                else:
                    logging.info(f"Wrong Unit board id{message['unit_id'] - 1}")
            # self.unit_semaphor.acquire()
            # self.unit_np_shm[0] = self.unit_np_shm[0] + 1
            # self.unit_semaphor.release()
            
def main():
    tcp_queue = queue.Queue(maxsize=128)
    main_func = CosmoMain(tcp_queue)
    main_func.config_file = Config("/home/pi/Projects/cosmo-m/config/config.ini")      # For VSC
    common = main_func.config_file.get_map('common')
    
    global MAXUNITBOARD, ADDRESS
    MAXUNITBOARD = common['MAXUNITBOARD']
    GPIOADDR = int(common['GPIOADDR'], 16)
    
    ip = common['HOST']
    port = common['PORT']    
    manager = Manager()
    
    can_fd_receive = canfd.CanFDReceive(logging, main_func)
    can_fd_receive.start()
    
    main_func.client = tcp_client(ip, port, tcp_queue, logging, main_func.i2cbus, GPIOADDR, shared_socket)
    main_func.client.start()                           #tcp client 시작
    
    can_fd_transmitte = canfd.CanFDTransmitte(logging, main_func)
    can_fd_transmitte.queue = manager.Queue(128)
    can_fd_transmitte.start()
    
    
    for i in range(MAXUNITBOARD):
        can_fd_receive.receive_queue.append(manager.Queue(128))
        main_func.command_queue.append(manager.Queue(128))
    
    # 숫자를 저장할 numpy 배열(1차원) 생성
    arr = np.array([i for i in range(MAXUNITBOARD*20)], dtype=np.int32) 
    # 공유 메모리 생성
    shm = shared_memory.SharedMemory(create=True, size=arr.nbytes)
    # 공유 메모리의 버퍼를 numpy 배열로 변환
    main_func.unit_np_shm = np.ndarray(arr.shape, dtype=arr.dtype, buffer=shm.buf)
    main_func.unit_semaphor = manager.Semaphore(1) 
    main_func.start()

    while True:
        if shared_socket[0] and not shared_socket[0]._closed:           #처음 서버에 연결 될 때까지 무한루프 실행
            with ProcessPoolExecutor(max_workers=5) as executor:
                unit_func = unit_board(logging, can_fd_transmitte.queue, shared_socket)
                furtures = {executor.submit(unit_func.unit_process, i, shm.name, main_func.unit_np_shm, 
                                            main_func.unit_semaphor, can_fd_receive.receive_queue[i],
                                            main_func.command_queue[i], main_func.config_file.get_map(f'unit_board{i+1}')) : i for i in range(MAXUNITBOARD)}

                for furture in as_completed(furtures):    
                    print("All Process is done")
                os.exit(1)
        else:
            time.sleep(1)
            print("Server is not Connected")
        
if __name__ == "__main__":
    main()
    