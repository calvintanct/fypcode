import time, socket, binascii, sys, numpy, math

class Suction(object):
    """docstring for Suction"""
    def __init__(self):
        self.UR5Host = "192.168.1.100"
        self.UR5Port = 500
        self.UR5Server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.UR5Server.bind((self.UR5Host,self.UR5Port))
        self.UR5Server.listen(1)
        self.connUR5,self.addrUR5 = self.UR5Server.accept()
        time.sleep(0.05)

    def on_suction(self):
        self.connUR5.send(bytes("on",'ascii'))
        time.sleep(0.1)

    def off_suction(self):
        self.connUR5.send(bytes("off",'ascii'))
        time.sleep(0.1)

    def sensor_suction(self):
        self.connUR5.send(bytes("read",'ascii'))
        time.sleep(0.1)
        receive=connUR10.recv(1000)
        return int(receive)




