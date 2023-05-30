#! /usr/bin/python3

import socket
import json
import time
import threading
import smbus

class TcpClientThread(threading.Thread):
    def __init__(self, ip, port, tcp_queue, logging, GPIOADDR, shared_object, i2c_semaphor):
        threading.Thread.__init__(self)
        self.daemon = True
        self.ip = ip
        self.port = port
        self.logging = logging
        self.tcp_queue = tcp_queue
        self.shared_object = shared_object
        self.GPIOADDR = GPIOADDR
        self.i2c_semaphor = i2c_semaphor
        
    def run(self):
            self.logging.info('클라이언트 동작')
            SERVER_ADDR = (self.ip, self.port)
            while True:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
                        client.connect(SERVER_ADDR)
                        self.logging.info(f'서버에 연결 되었습니다.{client}')
                        self.i2c_semaphor.acquire()
                        i2cbus = smbus.SMBus(1) 
                        i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF)
                        i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF)
                        i2cbus.close()
                        self.i2c_semaphor.release()
                        self.shared_object.insert(0, client)
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
