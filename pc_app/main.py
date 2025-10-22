#!/usr/bin/env python3
"""
Паяльная станция ZM-R5860 - PyQt5 GUI приложение
Версия 3.0 - совместимо с ESP32-S3 контроллером
"""

import sys
import json
import requests
import serial
import threading
import time
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                             QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox,
                             QCheckBox, QTabWidget, QTextEdit, QFileDialog,
                             QMessageBox, QProgressBar, QGroupBox, QSlider)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QPalette, QColor
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.dates as mdates


class ConnectionManager:
    """Менеджер подключений к паяльной станции"""
    
    def __init__(self):
        self.connection_type = None  # 'serial' или 'wifi'
        self.serial_conn = None
        self.wifi_ip = None
        self.connected = False
        
    def connect_serial(self, port, baudrate=115200):
        """Подключение через Serial порт"""
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            self.connection_type = 'serial'
            self.connected = True
            return True
        except Exception as e:
            print(f"Ошибка подключения к Serial: {e}")
            return False
    
    def connect_wifi(self, ip_address):
        """Подключение через WiFi"""
        try:
            # Проверяем доступность устройства
            response = requests.get(f"http://{ip_address}/status", timeout=2)
            if response.status_code == 200:
                self.wifi_ip = ip_address
                self.connection_type = 'wifi'
                self.connected = True
                return True
        except Exception as e:
            print(f"Ошибка подключения к WiFi: {e}")
        return False
    
    def disconnect(self):
        """Отключение от устройства"""
        if self.connection_type == 'serial' and self.serial_conn:
            self.serial_conn.close()
        self.connected = False
        self.connection_type = None
        self.serial_conn = None
        self.wifi_ip = None
    
    def send_command(self, endpoint, data=None, method='GET'):
        """Отправка команды на устройство"""
        if not self.connected:
            return None
            
        try:
            if self.connection_type == 'wifi':
                url = f"http://{self.wifi_ip}/{endpoint}"
                if method == 'GET':
                    response = requests.get(url, timeout=2)
                else:
                    response = requests.post(url, json=data, timeout=2)
                return response.json() if response.status_code == 200 else None
            elif self.connection_type == 'serial':
                # Для Serial пока простые команды
                command = f"{endpoint}\n"
                self.serial_conn.write(command.encode())
                response = self.serial_conn.readline().decode().strip()
                return response
        except Exception as e:
            print(f"Ошибка отправки команды: {e}")
            return None


class DataLogger(QThread):
    """Поток для логирования данных с устройства"""
    
    data_received = pyqtSignal(dict)
    
    def __init__(self, connection_manager):
        super().__init__()
        self.connection_manager = connection_manager
        self.running = False
        self.log_data = []
        
    def run(self):
        """Основной цикл логирования"""
        self.running = True
        while self.running:
            if self.connection_manager.connected:
                status = self.connection_manager.send_command('status')
                if status:
                    self.data_received.emit(status)
                    self.log_data.append({
                        'timestamp': datetime.now(),
                        'data': status
                    })
            time.sleep(0.3)  # Обновление каждые 300мс
    
    def stop(self):
        """Остановка логирования"""
        self.running = False


class TemperatureChart(FigureCanvas):
    """Виджет графика температур"""
    
    def __init__(self, parent=None):
        self.figure = Figure(figsize=(12, 6))
        super().__init__(self.figure)
        self.setParent(parent)
        
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title('Температуры в реальном времени')
        self.ax.set_xlabel('Время')
        self.ax.set_ylabel('Температура (°C)')
        self.ax.grid(True, alpha=0.3)
        
        self.timestamps = []
        self.temp_top = []
        self.temp_bottom = []
        self.temp_ir = []
        self.temp_external = []
        
        self.max_points = 100
        
    def update_data(self, data):
        """Обновление данных графика"""
        now = datetime.now()
        self.timestamps.append(now)
        self.temp_top.append(data.get('top', 0))
        self.temp_bottom.append(data.get('bottom', 0))
        self.temp_ir.append(data.get('ir', 0))
        self.temp_external.append(data.get('external', 0))
        
        # Ограничиваем количество точек
        if len(self.timestamps) > self.max_points:
            self.timestamps = self.timestamps[-self.max_points:]
            self.temp_top = self.temp_top[-self.max_points:]
            self.temp_bottom = self.temp_bottom[-self.max_points:]
            self.temp_ir = self.temp_ir[-self.max_points:]
            self.temp_external = self.temp_external[-self.max_points:]
        
        self.update_plot()
    
    def update_plot(self):
        """Обновление графика"""
        self.ax.clear()
        self.ax.set_title('Температуры в реальном времени')
        self.ax.set_xlabel('Время')
        self.ax.set_ylabel('Температура (°C)')
        self.ax.grid(True, alpha=0.3)
        
        if self.timestamps:
            self.ax.plot(self.timestamps, self.temp_top, 'r-', label='Верхний фен', linewidth=2)
            self.ax.plot(self.timestamps, self.temp_bottom, 'b-', label='Нижний фен', linewidth=2)
            self.ax.plot(self.timestamps, self.temp_ir, 'orange', label='IR-стол', linewidth=2)
            self.ax.plot(self.timestamps, self.temp_external, 'g-', label='Внешняя ТС', linewidth=2)
            
            self.ax.legend()
            self.ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
            self.ax.xaxis.set_major_locator(mdates.SecondLocator(interval=10))
            plt.setp(self.ax.xaxis.get_majorticklabels(), rotation=45)
        
        self.figure.tight_layout()
        self.draw()


class SolderingStationGUI(QMainWindow):
    """Основное окно приложения"""
    
    def __init__(self):
        super().__init__()
        self.connection_manager = ConnectionManager()
        self.data_logger = DataLogger(self.connection_manager)
        self.data_logger.data_received.connect(self.update_temperatures)
        
        self.init_ui()
        self.setup_connections()
        
    def init_ui(self):
        """Инициализация интерфейса"""
        self.setWindowTitle('Паяльная станция ZM-R5860 v3.0')
        self.setGeometry(100, 100, 1400, 900)
        
        # Центральный виджет
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Основной layout
        main_layout = QVBoxLayout(central_widget)
        
        # Создаем вкладки
        tab_widget = QTabWidget()
        main_layout.addWidget(tab_widget)
        
        # Вкладка подключения
        self.connection_tab = self.create_connection_tab()
        tab_widget.addTab(self.connection_tab, "Подключение")
        
        # Вкладка мониторинга
        self.monitoring_tab = self.create_monitoring_tab()
        tab_widget.addTab(self.monitoring_tab, "Мониторинг")
        
        # Вкладка профилей
        self.profiles_tab = self.create_profiles_tab()
        tab_widget.addTab(self.profiles_tab, "Профили")
        
        # Вкладка управления
        self.control_tab = self.create_control_tab()
        tab_widget.addTab(self.control_tab, "Управление")
        
        # Статус бар
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Не подключено")
        
    def create_connection_tab(self):
        """Создание вкладки подключения"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Группа подключения
        connection_group = QGroupBox("Подключение к устройству")
        connection_layout = QGridLayout(connection_group)
        
        # Serial подключение
        connection_layout.addWidget(QLabel("Serial порт:"), 0, 0)
        self.serial_port_input = QLineEdit("COM3")
        connection_layout.addWidget(self.serial_port_input, 0, 1)
        
        self.connect_serial_btn = QPushButton("Подключиться (Serial)")
        connection_layout.addWidget(self.connect_serial_btn, 0, 2)
        
        # WiFi подключение
        connection_layout.addWidget(QLabel("WiFi IP:"), 1, 0)
        self.wifi_ip_input = QLineEdit("192.168.4.1")
        connection_layout.addWidget(self.wifi_ip_input, 1, 1)
        
        self.connect_wifi_btn = QPushButton("Подключиться (WiFi)")
        connection_layout.addWidget(self.connect_wifi_btn, 1, 2)
        
        # Отключение
        self.disconnect_btn = QPushButton("Отключиться")
        self.disconnect_btn.setEnabled(False)
        connection_layout.addWidget(self.disconnect_btn, 2, 2)
        
        layout.addWidget(connection_group)
        
        # Статус подключения
        self.connection_status = QLabel("Статус: Не подключено")
        self.connection_status.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.connection_status)
        
        layout.addStretch()
        return tab
    
    def create_monitoring_tab(self):
        """Создание вкладки мониторинга"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Текущие температуры
        temp_group = QGroupBox("Текущие температуры")
        temp_layout = QGridLayout(temp_group)
        
        self.temp_labels = {}
        sensors = ['top', 'bottom', 'ir', 'external']
        sensor_names = ['Верхний фен', 'Нижний фен', 'IR-стол', 'Внешняя ТС']
        
        for i, (sensor, name) in enumerate(zip(sensors, sensor_names)):
            temp_layout.addWidget(QLabel(f"{name}:"), i, 0)
            self.temp_labels[sensor] = QLabel("--.- °C")
            self.temp_labels[sensor].setFont(QFont("Arial", 12, QFont.Bold))
            temp_layout.addWidget(self.temp_labels[sensor], i, 1)
        
        layout.addWidget(temp_group)
        
        # График температур
        self.temperature_chart = TemperatureChart()
        layout.addWidget(self.temperature_chart)
        
        return tab
    
    def create_profiles_tab(self):
        """Создание вкладки профилей"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Управление профилями
        profile_group = QGroupBox("Управление профилями")
        profile_layout = QVBoxLayout(profile_group)
        
        # Название профиля
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Название профиля:"))
        self.profile_name_input = QLineEdit("Lead-Free BGA")
        name_layout.addWidget(self.profile_name_input)
        profile_layout.addLayout(name_layout)
        
        # Лимит температуры
        limit_layout = QHBoxLayout()
        limit_layout.addWidget(QLabel("Лимит температуры:"))
        self.temp_limit_spin = QSpinBox()
        self.temp_limit_spin.setRange(100, 400)
        self.temp_limit_spin.setValue(280)
        self.temp_limit_spin.setSuffix(" °C")
        limit_layout.addWidget(self.temp_limit_spin)
        profile_layout.addLayout(limit_layout)
        
        # Фазы профиля
        phases_group = QGroupBox("Фазы профиля")
        phases_layout = QVBoxLayout(phases_group)
        
        # Кнопки управления фазами
        phases_buttons = QHBoxLayout()
        self.add_phase_btn = QPushButton("+ Добавить фазу")
        self.remove_phase_btn = QPushButton("- Удалить фазу")
        phases_buttons.addWidget(self.add_phase_btn)
        phases_buttons.addWidget(self.remove_phase_btn)
        phases_buttons.addStretch()
        phases_layout.addLayout(phases_buttons)
        
        # Контейнер для фаз
        self.phases_container = QWidget()
        self.phases_layout = QVBoxLayout(self.phases_container)
        phases_layout.addWidget(self.phases_container)
        
        profile_layout.addWidget(phases_group)
        
        # Кнопки сохранения/загрузки
        save_load_layout = QHBoxLayout()
        self.save_profile_btn = QPushButton("Сохранить профиль")
        self.load_profile_btn = QPushButton("Загрузить профиль")
        save_load_layout.addWidget(self.save_profile_btn)
        save_load_layout.addWidget(self.load_profile_btn)
        save_load_layout.addStretch()
        profile_layout.addLayout(save_load_layout)
        
        layout.addWidget(profile_group)
        
        # Инициализация фаз по умолчанию
        self.init_default_phases()
        
        return tab
    
    def create_control_tab(self):
        """Создание вкладки управления"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Управление процессом
        control_group = QGroupBox("Управление процессом")
        control_layout = QVBoxLayout(control_group)
        
        # Статус процесса
        status_layout = QHBoxLayout()
        status_layout.addWidget(QLabel("Статус процесса:"))
        self.process_status_label = QLabel("IDLE")
        self.process_status_label.setStyleSheet("color: blue; font-weight: bold;")
        status_layout.addWidget(self.process_status_label)
        status_layout.addStretch()
        control_layout.addLayout(status_layout)
        
        # Текущая фаза
        phase_layout = QHBoxLayout()
        phase_layout.addWidget(QLabel("Текущая фаза:"))
        self.current_phase_label = QLabel("-")
        phase_layout.addWidget(self.current_phase_label)
        phase_layout.addWidget(QLabel("Осталось:"))
        self.remain_time_label = QLabel("-")
        phase_layout.addWidget(self.remain_time_label)
        phase_layout.addStretch()
        control_layout.addLayout(phase_layout)
        
        # Выходы
        outputs_layout = QHBoxLayout()
        outputs_layout.addWidget(QLabel("Выходы:"))
        self.outputs_label = QLabel("0/0/0 %")
        outputs_layout.addWidget(self.outputs_label)
        outputs_layout.addStretch()
        control_layout.addLayout(outputs_layout)
        
        # Кнопки управления
        buttons_layout = QHBoxLayout()
        self.start_btn = QPushButton("СТАРТ")
        self.start_btn.setStyleSheet("QPushButton { background-color: green; color: white; font-weight: bold; }")
        self.stop_btn = QPushButton("СТОП")
        self.stop_btn.setStyleSheet("QPushButton { background-color: red; color: white; font-weight: bold; }")
        self.fan_btn = QPushButton("Вентилятор")
        
        buttons_layout.addWidget(self.start_btn)
        buttons_layout.addWidget(self.stop_btn)
        buttons_layout.addWidget(self.fan_btn)
        buttons_layout.addStretch()
        control_layout.addLayout(buttons_layout)
        
        layout.addWidget(control_group)
        
        # Логирование
        log_group = QGroupBox("Логирование")
        log_layout = QVBoxLayout(log_group)
        
        log_buttons_layout = QHBoxLayout()
        self.start_log_btn = QPushButton("Начать логирование")
        self.stop_log_btn = QPushButton("Остановить логирование")
        self.export_log_btn = QPushButton("Экспорт лога")
        self.export_log_btn.setEnabled(False)
        
        log_buttons_layout.addWidget(self.start_log_btn)
        log_buttons_layout.addWidget(self.stop_log_btn)
        log_buttons_layout.addWidget(self.export_log_btn)
        log_buttons_layout.addStretch()
        log_layout.addLayout(log_buttons_layout)
        
        # Текстовое поле для логов
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)
        
        layout.addWidget(log_group)
        
        layout.addStretch()
        return tab
    
    def init_default_phases(self):
        """Инициализация фаз по умолчанию"""
        phases = [
            {"name": "Preheat", "target": 165, "duration": 90, "kp": 2.0, "ki": 0.08, "use_top": True, "use_bottom": True, "use_ir": True},
            {"name": "Soak", "target": 190, "duration": 60, "kp": 2.1, "ki": 0.09, "use_top": True, "use_bottom": True, "use_ir": True},
            {"name": "Reflow", "target": 255, "duration": 30, "kp": 2.5, "ki": 0.10, "use_top": True, "use_bottom": False, "use_ir": False},
            {"name": "Cool", "target": 100, "duration": 90, "kp": 1.0, "ki": 0.05, "use_top": False, "use_bottom": False, "use_ir": False}
        ]
        
        for phase_data in phases:
            self.add_phase_widget(phase_data)
    
    def add_phase_widget(self, phase_data=None):
        """Добавление виджета фазы"""
        if phase_data is None:
            phase_data = {"name": "Новая фаза", "target": 150, "duration": 60, "kp": 2.0, "ki": 0.08, "use_top": True, "use_bottom": True, "use_ir": True}
        
        phase_widget = QGroupBox(f"Фаза {self.phases_layout.count() + 1}")
        phase_layout = QGridLayout(phase_widget)
        
        # Название фазы
        phase_layout.addWidget(QLabel("Название:"), 0, 0)
        name_input = QLineEdit(phase_data["name"])
        phase_layout.addWidget(name_input, 0, 1)
        
        # Целевая температура
        phase_layout.addWidget(QLabel("Цель (°C):"), 0, 2)
        target_spin = QSpinBox()
        target_spin.setRange(50, 400)
        target_spin.setValue(phase_data["target"])
        phase_layout.addWidget(target_spin, 0, 3)
        
        # Длительность
        phase_layout.addWidget(QLabel("Время (сек):"), 1, 0)
        duration_spin = QSpinBox()
        duration_spin.setRange(1, 3600)
        duration_spin.setValue(phase_data["duration"])
        phase_layout.addLayout(duration_spin, 1, 1)
        
        # Kp
        phase_layout.addWidget(QLabel("Kp:"), 1, 2)
        kp_spin = QDoubleSpinBox()
        kp_spin.setRange(0.1, 10.0)
        kp_spin.setDecimals(2)
        kp_spin.setValue(phase_data["kp"])
        phase_layout.addWidget(kp_spin, 1, 3)
        
        # Ki
        phase_layout.addWidget(QLabel("Ki:"), 2, 0)
        ki_spin = QDoubleSpinBox()
        ki_spin.setRange(0.001, 1.0)
        ki_spin.setDecimals(3)
        ki_spin.setValue(phase_data["ki"])
        phase_layout.addWidget(ki_spin, 2, 1)
        
        # Использование нагревателей
        use_layout = QHBoxLayout()
        use_top_check = QCheckBox("Верхний")
        use_top_check.setChecked(phase_data["use_top"])
        use_bottom_check = QCheckBox("Нижний")
        use_bottom_check.setChecked(phase_data["use_bottom"])
        use_ir_check = QCheckBox("IR")
        use_ir_check.setChecked(phase_data["use_ir"])
        
        use_layout.addWidget(use_top_check)
        use_layout.addWidget(use_bottom_check)
        use_layout.addWidget(use_ir_check)
        use_layout.addStretch()
        phase_layout.addLayout(use_layout, 2, 2, 1, 2)
        
        # Сохраняем ссылки на виджеты
        phase_widget.name_input = name_input
        phase_widget.target_spin = target_spin
        phase_widget.duration_spin = duration_spin
        phase_widget.kp_spin = kp_spin
        phase_widget.ki_spin = ki_spin
        phase_widget.use_top_check = use_top_check
        phase_widget.use_bottom_check = use_bottom_check
        phase_widget.use_ir_check = use_ir_check
        
        self.phases_layout.addWidget(phase_widget)
    
    def setup_connections(self):
        """Настройка соединений сигналов и слотов"""
        # Подключение
        self.connect_serial_btn.clicked.connect(self.connect_serial)
        self.connect_wifi_btn.clicked.connect(self.connect_wifi)
        self.disconnect_btn.clicked.connect(self.disconnect)
        
        # Управление фазами
        self.add_phase_btn.clicked.connect(lambda: self.add_phase_widget())
        self.remove_phase_btn.clicked.connect(self.remove_phase)
        
        # Профили
        self.save_profile_btn.clicked.connect(self.save_profile)
        self.load_profile_btn.clicked.connect(self.load_profile)
        
        # Управление процессом
        self.start_btn.clicked.connect(self.start_process)
        self.stop_btn.clicked.connect(self.stop_process)
        self.fan_btn.clicked.connect(self.toggle_fan)
        
        # Логирование
        self.start_log_btn.clicked.connect(self.start_logging)
        self.stop_log_btn.clicked.connect(self.stop_logging)
        self.export_log_btn.clicked.connect(self.export_log)
    
    def connect_serial(self):
        """Подключение через Serial"""
        port = self.serial_port_input.text()
        if self.connection_manager.connect_serial(port):
            self.update_connection_status("Подключено (Serial)")
            self.status_bar.showMessage(f"Подключено к {port}")
        else:
            QMessageBox.warning(self, "Ошибка", "Не удалось подключиться к Serial порту")
    
    def connect_wifi(self):
        """Подключение через WiFi"""
        ip = self.wifi_ip_input.text()
        if self.connection_manager.connect_wifi(ip):
            self.update_connection_status("Подключено (WiFi)")
            self.status_bar.showMessage(f"Подключено к {ip}")
        else:
            QMessageBox.warning(self, "Ошибка", "Не удалось подключиться к устройству по WiFi")
    
    def disconnect(self):
        """Отключение от устройства"""
        self.connection_manager.disconnect()
        self.update_connection_status("Не подключено")
        self.status_bar.showMessage("Отключено")
    
    def update_connection_status(self, status):
        """Обновление статуса подключения"""
        self.connection_status.setText(f"Статус: {status}")
        connected = status != "Не подключено"
        
        self.connect_serial_btn.setEnabled(not connected)
        self.connect_wifi_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        
        # Включаем/выключаем элементы управления
        self.start_btn.setEnabled(connected)
        self.stop_btn.setEnabled(connected)
        self.fan_btn.setEnabled(connected)
        self.save_profile_btn.setEnabled(connected)
        self.load_profile_btn.setEnabled(connected)
    
    def update_temperatures(self, data):
        """Обновление отображения температур"""
        # Обновляем метки температур
        for sensor, label in self.temp_labels.items():
            temp = data.get(sensor, 0)
            label.setText(f"{temp:.1f} °C")
        
        # Обновляем график
        self.temperature_chart.update_data(data)
        
        # Обновляем статус процесса
        state = data.get('state', 'IDLE')
        self.process_status_label.setText(state)
        self.current_phase_label.setText(data.get('phase', '-'))
        self.remain_time_label.setText(f"{data.get('remain', 0)} сек")
        self.outputs_label.setText(f"{data.get('outTop', 0)}/{data.get('outBottom', 0)}/{data.get('outIR', 0)} %")
        
        # Цветовая индикация статуса
        if state == "RUNNING":
            self.process_status_label.setStyleSheet("color: green; font-weight: bold;")
        elif state == "ABORTED":
            self.process_status_label.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.process_status_label.setStyleSheet("color: blue; font-weight: bold;")
    
    def remove_phase(self):
        """Удаление последней фазы"""
        if self.phases_layout.count() > 0:
            item = self.phases_layout.takeAt(self.phases_layout.count() - 1)
            if item.widget():
                item.widget().deleteLater()
    
    def save_profile(self):
        """Сохранение профиля на устройство"""
        if not self.connection_manager.connected:
            QMessageBox.warning(self, "Ошибка", "Не подключено к устройству")
            return
        
        # Собираем данные профиля
        profile_data = {
            "name": self.profile_name_input.text(),
            "overLimitC": self.temp_limit_spin.value(),
            "n": self.phases_layout.count(),
            "phases": []
        }
        
        for i in range(self.phases_layout.count()):
            phase_widget = self.phases_layout.itemAt(i).widget()
            phase_data = {
                "name": phase_widget.name_input.text(),
                "targetC": phase_widget.target_spin.value(),
                "seconds": phase_widget.duration_spin.value(),
                "Kp": phase_widget.kp_spin.value(),
                "Ki": phase_widget.ki_spin.value(),
                "useTop": phase_widget.use_top_check.isChecked(),
                "useBottom": phase_widget.use_bottom_check.isChecked(),
                "useIR": phase_widget.use_ir_check.isChecked()
            }
            profile_data["phases"].append(phase_data)
        
        # Отправляем на устройство
        result = self.connection_manager.send_command('preset', profile_data, 'POST')
        if result:
            QMessageBox.information(self, "Успех", "Профиль сохранен")
        else:
            QMessageBox.warning(self, "Ошибка", "Не удалось сохранить профиль")
    
    def load_profile(self):
        """Загрузка профиля с устройства"""
        if not self.connection_manager.connected:
            QMessageBox.warning(self, "Ошибка", "Не подключено к устройству")
            return
        
        # Получаем профиль с устройства
        profile_data = self.connection_manager.send_command('preset')
        if profile_data:
            self.profile_name_input.setText(profile_data.get('name', ''))
            self.temp_limit_spin.setValue(profile_data.get('overLimitC', 280))
            
            # Очищаем существующие фазы
            while self.phases_layout.count():
                item = self.phases_layout.takeAt(0)
                if item.widget():
                    item.widget().deleteLater()
            
            # Добавляем фазы из профиля
            for phase_data in profile_data.get('phases', []):
                self.add_phase_widget(phase_data)
            
            QMessageBox.information(self, "Успех", "Профиль загружен")
        else:
            QMessageBox.warning(self, "Ошибка", "Не удалось загрузить профиль")
    
    def start_process(self):
        """Запуск процесса пайки"""
        if not self.connection_manager.connected:
            QMessageBox.warning(self, "Ошибка", "Не подключено к устройству")
            return
        
        result = self.connection_manager.send_command('start', method='POST')
        if result:
            self.log_message("Процесс пайки запущен")
        else:
            QMessageBox.warning(self, "Ошибка", "Не удалось запустить процесс")
    
    def stop_process(self):
        """Остановка процесса пайки"""
        if not self.connection_manager.connected:
            return
        
        result = self.connection_manager.send_command('stop', method='POST')
        if result:
            self.log_message("Процесс пайки остановлен")
    
    def toggle_fan(self):
        """Переключение вентилятора"""
        if not self.connection_manager.connected:
            return
        
        result = self.connection_manager.send_command('fan', method='POST')
        if result:
            self.log_message(f"Вентилятор: {result}")
    
    def start_logging(self):
        """Начало логирования"""
        self.data_logger.start()
        self.start_log_btn.setEnabled(False)
        self.stop_log_btn.setEnabled(True)
        self.log_message("Логирование начато")
    
    def stop_logging(self):
        """Остановка логирования"""
        self.data_logger.stop()
        self.start_log_btn.setEnabled(True)
        self.stop_log_btn.setEnabled(False)
        self.export_log_btn.setEnabled(True)
        self.log_message("Логирование остановлено")
    
    def export_log(self):
        """Экспорт лога в файл"""
        if not self.data_logger.log_data:
            QMessageBox.information(self, "Информация", "Нет данных для экспорта")
            return
        
        filename, _ = QFileDialog.getSaveFileName(self, "Сохранить лог", f"soldering_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "CSV файлы (*.csv)")
        
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write("Время,Верхний фен,Нижний фен,IR-стол,Внешняя ТС,Фаза,Осталось\n")
                    for entry in self.data_logger.log_data:
                        data = entry['data']
                        f.write(f"{entry['timestamp'].strftime('%Y-%m-%d %H:%M:%S')},{data.get('top', 0):.1f},{data.get('bottom', 0):.1f},{data.get('ir', 0):.1f},{data.get('external', 0):.1f},{data.get('phase', '-')},{data.get('remain', 0)}\n")
                
                QMessageBox.information(self, "Успех", f"Лог сохранен в {filename}")
            except Exception as e:
                QMessageBox.warning(self, "Ошибка", f"Не удалось сохранить лог: {e}")
    
    def log_message(self, message):
        """Добавление сообщения в лог"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_text.append(f"[{timestamp}] {message}")
        self.log_text.ensureCursorVisible()


def main():
    """Главная функция"""
    app = QApplication(sys.argv)
    app.setApplicationName("Паяльная станция ZM-R5860")
    app.setApplicationVersion("3.0")
    
    window = SolderingStationGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
