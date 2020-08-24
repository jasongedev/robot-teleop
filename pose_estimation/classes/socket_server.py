import socket

class SocketServer:
    def __init__(self):
       
        self.x = 3

    def send_data(self, data):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('127.0.0.1', 8082))
        self.sock.sendall(str(data).encode())
        #print(self.sock.recv(1024).decode())
        self.sock.close
