#! /usr/bin/python3

import socket
import json
import time
import threading

data = {
    "name": "hello, I am Tom.",
    "age": 10,
    "info": "sample is simple."
}

class TcpClientThread(threading.Thread):
    def __init__(self, ip, port, tcp_queue, logging, i2cbus, GPIOADDR, shared_socket):
        threading.Thread.__init__(self)
        self.daemon = True
        self.ip = ip
        self.port = port
        self.logging = logging
        self.tcp_queue = tcp_queue
        self.shared_socket = shared_socket
        self.i2cbus = i2cbus
        self.GPIOADDR = GPIOADDR
        
    def run(self):
            self.logging.info('클라이언트 동작')
            SERVER_ADDR = (self.ip, self.port)
            while True:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
                        client.connect(SERVER_ADDR)
                        self.logging.info(f'서버에 연결 되었습니다.{client}')
                        self.i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF)
                        self.i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF)
                        self.shared_socket.insert(0, client)
                        while True:
                            data = json.loads(client.recv(1024).decode('UTF-8'))
                            self.tcp_queue.put(data)
                            # self.client.sendall(bytes(json.dumps({"status":"success!"}), 'UTF-8'))
                except Exception as e:
                    client.close()
                    time.sleep(0.5)
                    self.i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0xFF)
                    self.i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0xFF)
                    time.sleep(0.5)
                    self.i2cbus.write_byte_data(self.GPIOADDR, 0x12, 0x00)
                    self.i2cbus.write_byte_data(self.GPIOADDR, 0x13, 0x00)
                    print(e)
