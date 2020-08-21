import json
import socketserver
import socket
import threading

def ServerRequestHandlerFactory(init_args):
    class ServerRequestHandler(socketserver.BaseRequestHandler):
        def __init__(self, *args, **kwargs):
            #self.joint_angles_data = init_args.angles
            super(ServerRequestHandler, self).__init__(*args, **kwargs)
        def handle(self):
            self.request.send_data(data)
            recdata = self.request.recv(1024)
            print(recdata)
            return
    return ServerRequestHandler

class SocketServer:
    def __init__(self):
       
        self.x = 3

    def send_data(self, data):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('127.0.0.1', 8083))
        self.sock.sendall(str(data).encode())
        #print(self.sock.recv(1024).decode())
        self.sock.close
