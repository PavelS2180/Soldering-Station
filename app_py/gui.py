import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QRadioButton, \
    QHBoxLayout, QLineEdit
from connection import Connection
from commands import Commands
from temperature import Temperature


class SolderingStationGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.connection = Connection()
        self.commands = Commands(self.connection)
        self.temperature = Temperature(self.connection)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Connection type selection
        connection_layout = QHBoxLayout()
        self.serial_radio = QRadioButton("Serial", self)
        self.wifi_radio = QRadioButton("WiFi", self)
        self.serial_radio.toggled.connect(self.set_connection_type)
        self.wifi_radio.toggled.connect(self.set_connection_type)
        connection_layout.addWidget(self.serial_radio)
        connection_layout.addWidget(self.wifi_radio)
        layout.addLayout(connection_layout)

        # Serial port input
        self.serial_port_input = QLineEdit(self)
        self.serial_port_input.setPlaceholderText("Enter Serial Port (e.g., COM3)")
        self.serial_port_input.setDisabled(True)
        layout.addWidget(self.serial_port_input)

        # WiFi IP address input
        self.wifi_ip_input = QLineEdit(self)
        self.wifi_ip_input.setPlaceholderText("Enter WiFi IP Address (e.g., 192.168.1.100)")
        self.wifi_ip_input.setDisabled(True)
        layout.addWidget(self.wifi_ip_input)

        self.connect_button = QPushButton('Connect', self)
        self.connect_button.clicked.connect(self.connect_to_station)
        layout.addWidget(self.connect_button)

        self.tempLabelTop = QLabel('Температура верхнего нагревателя: 0°C', self)
        layout.addWidget(self.tempLabelTop)

        self.tempLabelBottom = QLabel('Температура нижнего нагревателя: 0°C', self)
        layout.addWidget(self.tempLabelBottom)

        self.tempLabelPreheat = QLabel('Температура преднагревателя: 0°C', self)
        layout.addWidget(self.tempLabelPreheat)

        self.profileComboBox = QComboBox(self)
        self.profileComboBox.addItems(['Профиль 1', 'Профиль 2', 'Профиль 3'])
        layout.addWidget(self.profileComboBox)

        self.startButton = QPushButton('Старт', self)
        self.startButton.clicked.connect(self.start_soldering)
        layout.addWidget(self.startButton)

        self.stopButton = QPushButton('Стоп', self)
        self.stopButton.clicked.connect(self.stop_soldering)
        layout.addWidget(self.stopButton)

        self.setLayout(layout)
        self.setWindowTitle('Паяльная станция')
        self.show()

    def set_connection_type(self):
        if self.serial_radio.isChecked():
            self.connection.set_connection_type('serial')
            self.serial_port_input.setDisabled(False)
            self.wifi_ip_input.setDisabled(True)
        elif self.wifi_radio.isChecked():
            self.connection.set_connection_type('wifi')
            self.serial_port_input.setDisabled(True)
            self.wifi_ip_input.setDisabled(False)

    def connect_to_station(self):
        if self.connection.connection_type == 'serial':
            port = self.serial_port_input.text()
            if not self.connection.connect_serial(port):
                print("Failed to connect to serial port.")
        elif self.connection.connection_type == 'wifi':
            ip_address = self.wifi_ip_input.text()
            if not self.connection.connect_wifi(ip_address):
                print("Failed to connect to WiFi.")
        self.update_temperatures()

    def start_soldering(self):
        profile = self.profileComboBox.currentText()
        self.commands.start_soldering(profile)

    def stop_soldering(self):
        self.commands.stop_soldering()

    def update_temperatures(self):
        self.commands.get_temperature_data()
        self.temperature.update_temperatures()
        self.tempLabelTop.setText(f'Температура верхнего нагревателя: {self.temperature.get_top_temp()}°C')
        self.tempLabelBottom.setText(f'Температура нижнего нагревателя: {self.temperature.get_bottom_temp()}°C')
        self.tempLabelPreheat.setText(f'Температура преднагревателя: {self.temperature.get_preheat_temp()}°C')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = SolderingStationGUI()
    sys.exit(app.exec_())
