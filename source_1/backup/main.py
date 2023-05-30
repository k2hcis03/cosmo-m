#! /usr/bin/python3

import smbus
import os
import can
import time

from logger import logger as logging
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing import Process, current_process, Queue, Manager
import threading
import time
import canfd
import queue
from concurrent.futures import wait

address = 0x20
MAXUNIT = 1


class UnitBoard:
    def __init__(self, logging, transmitte_queue) -> None:
        self.transmitte_queue = transmitte_queue
        self.receive_queue = None
        self.logging = logging
        
    def unit_process(self, n, receive_queue):
        self.logging.info(f'Process {os.getpid()} and {n} maked')
        self.receive_queue = receive_queue
        # msg = can.Message(is_extended_id=False, arbitration_id=0x300+0x1f, data=[0, 1, 2, 3, 4, 5, 6, 7])
        # self.transmitte_queue.put(msg)
        while True:
            time.sleep(1)
            msg = can.Message(is_extended_id=False, arbitration_id=0x300+0x1f, is_fd = True, bitrate_switch = True, data=[0xF2, 0x11, 0x02, 0x00, 0x10, 0x1F, 0xF3])
            self.transmitte_queue.put(msg)
        
class CosmoMain:
    def __init__(self) -> None:
        os.system("sudo ip link set can0 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on")
        os.system("sudo ifconfig can0 txqueuelen 65536")
        filters = [
            {"can_id": 0x31F, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x300, "can_mask": 0x7FF, "extended": False},
        ]
        self.can0 = can.interface.Bus(rx_fifo_size = 8192, channel = 'can0', bustype = 'socketcan', bitrate_switch = True, bitrate = 1000000, data_bitrate = 8000000, fd = True, can_filters=filters)  # socketcan_native
        
        self.i2cbus = smbus.SMBus(1) 
        self.i2cbus.write_byte_data(address, 0x00, 0x00)        # OUTPUT
        self.i2cbus.write_byte_data(address, 0x01, 0x00)        # OUTPUT
        self.i2cbus.write_byte_data(address, 0x12, 0xFF)
        self.i2cbus.write_byte_data(address, 0x13, 0xFF)

        self.transmitte_queue = None
        self.receive_queue = []
        
    def cosmo_m_main_job(self):
        while True:
            while  not self.can_send_queue.empty():
                message = self.transmitte_queue.get()
                logging.info(message)
                self.can0.send(message)
                
def main():
    main_func = CosmoMain()
    manager = Manager()
    main_func.transmitte_queue = manager.Queue()
    
    can_fd_receive = canfd.CanFDReceive(logging, main_func)
    can_fd_receive.start()
    
    can_fd_transmitte = canfd.CanFDTransmitte(logging, main_func)
    can_fd_transmitte.start()

    for i in range(MAXUNIT):
        main_func.receive_queue.append(manager.Queue())
    
    with ThreadPoolExecutor(max_workers=2) as executor:
        future = executor.submit(main_func.cosmo_m_main_job)
        
    with ProcessPoolExecutor(max_workers=5) as executor:
        unit_func = UnitBoard(logging, main_func.transmitte_queue)
        furtures = {executor.submit(unit_func.unit_process, i, main_func.receive_queue[i]) : i for i in range(MAXUNIT)}

        for furture in as_completed(furtures):    
            print("All Process is done")
        
if __name__ == "__main__":
    main()
    