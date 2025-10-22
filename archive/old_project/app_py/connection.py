import socket
import serial

class Connection:
    """
    Класс для управления подключением к паяльной станции.
    Поддерживает подключение через Serial-порт или WiFi.
    """
    def __init__(self):
        """Инициализация параметров подключения"""
        self.connection_type = None  # Тип подключения: 'serial' или 'wifi'
        self.serial_conn = None      # Объект Serial-соединения
        self.wifi_conn = None        # Объект сокета WiFi-соединения

    def set_connection_type(self, connection_type):
        """Установка типа подключения: 'serial' или 'wifi'"""
        self.connection_type = connection_type

    def connect_serial(self, port):
        """
        Подключение через Serial-порт
        
        Args:
            port (str): Имя порта (например, 'COM3')
            
        Returns:
            bool: Успешность подключения
        """
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=1)
            return True
        except Exception as e:
            print(f"Error connecting to serial port: {e}")
            return False

    def connect_wifi(self, ip_address):
        """
        Подключение через WiFi
        
        Args:
            ip_address (str): IP-адрес паяльной станции
            
        Returns:
            bool: Успешность подключения
        """
        try:
            self.wifi_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.wifi_conn.connect((ip_address, 80))  # Подключение по порту 80
            return True
        except Exception as e:
            print(f"Error connecting to WiFi: {e}")
            return False

    def send(self, message):
        """
        Отправка сообщения на паяльную станцию
        
        Args:
            message (str): Команда для отправки
        """
        # Добавляем перевод строки и кодируем сообщение
        encoded_message = (message + '\n').encode()
        
        if self.connection_type == 'serial' and self.serial_conn:
            self.serial_conn.write(encoded_message)
        elif self.connection_type == 'wifi' and self.wifi_conn:
            self.wifi_conn.sendall(encoded_message)

    def receive(self):
        """
        Получение данных от паяльной станции
        
        Returns:
            str: Полученные данные или пустая строка при ошибке
        """
        if self.connection_type == 'serial' and self.serial_conn:
            return self.serial_conn.readline().decode()
        elif self.connection_type == 'wifi' and self.wifi_conn:
            return self.wifi_conn.recv(1024).decode()
        return ""
