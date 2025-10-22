class Commands:
    """
    Класс для работы с командами паяльной станции.
    Обеспечивает запуск/остановку процесса пайки и получение данных о температуре.
    """
    def __init__(self, connection):
        """Инициализация с объектом соединения"""
        self.connection = connection
        
        # Температурные профили пайки для быстрого доступа
        self.profiles = {
            'Профиль 1': {'top': 160, 'bottom': 170, 'preheat': 180},
            'Профиль 2': {'top': 170, 'bottom': 180, 'preheat': 190},
            'Профиль 3': {'top': 180, 'bottom': 190, 'preheat': 200}
        }

    def start_soldering(self, profile):
        """
        Запуск процесса пайки с выбранным профилем
        
        Args:
            profile (str): Название профиля ('Профиль 1', 'Профиль 2', 'Профиль 3')
        """
        if profile in self.profiles:
            # Отправляем команды установки температуры для каждого нагревателя
            # A - верхний нагреватель, B - нижний, P - преднагреватель
            temps = self.profiles[profile]
            self.connection.send(f"A{temps['top']}")
            self.connection.send(f"B{temps['bottom']}")
            self.connection.send(f"P{temps['preheat']}")

    def stop_soldering(self):
        """Остановка процесса пайки (выключение всех нагревателей)"""
        # Отправляем команды выключения нагревателей
        # T - выключить верхний, U - нижний, H - преднагреватель
        self.connection.send('T')  # Выключение верхнего нагревателя
        self.connection.send('U')  # Выключение нижнего нагревателя
        self.connection.send('H')  # Выключение преднагревателя

    def get_temperature_data(self):
        """
        Запрос данных о текущей температуре нагревателей
        
        Returns:
            str: Ответ от устройства с данными о температуре
        """
        self.connection.send('GET_TEMP')  # Команда запроса температуры
        return self.connection.receive()   # Возвращаем полученные данные
