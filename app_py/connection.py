import socket
import serial

class Connection:
    def __init__(self):
        self.connection_type = None
        self.serial_conn = None
        self.wifi_conn = None

    def set_connection_type(self, connection_type):
        self.connection_type = connection_type

    def connect_serial(self, port):
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=1)
            return True
        except Exception as e:
            print(f"Error connecting to serial port: {e}")
            return False

    def connect_wifi(self, ip_address):
        try:
            self.wifi_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.wifi_conn.connect((ip_address, 80))
            return True
        except Exception as e:
            print(f"Error connecting to WiFi: {e}")
            return False

    def send(self, message):
        if self.connection_type == 'serial' and self.serial_conn:
            self.serial_conn.write((message + '\n').encode())
        elif self.connection_type == 'wifi' and self.wifi_conn:
            self.wifi_conn.sendall((message + '\n').encode())

    def receive(self):
        if self.connection_type == 'serial' and self.serial_conn:
            return self.serial_conn.readline().decode()
        elif self.connection_type == 'wifi' and self.wifi_conn:
            return self.wifi_conn.recv(1024).decode()
        return ""
