class Temperature:
    """
    Класс для работы с температурными данными паяльной станции.
    Хранит и обновляет информацию о температуре нагревателей.
    """
    def __init__(self, connection):
        """
        Инициализация объекта для работы с температурой
        
        Args:
            connection: Объект соединения с паяльной станцией
        """
        self.connection = connection
        # Начальные значения температуры
        self.top_temp = 0      # Верхний нагреватель
        self.bottom_temp = 0   # Нижний нагреватель
        self.preheat_temp = 0  # Преднагреватель

    def update_temperatures(self):
        """
        Обновление значений температуры из данных, полученных от станции.
        Парсит строку с температурами, разделенными запятыми.
        """
        data = self.connection.receive()
        if data:
            temps = data.split(',')
            # Проверяем, что получили три значения (для трех нагревателей)
            if len(temps) == 3:
                self.top_temp = temps[0]     # Первое значение - верхний нагреватель
                self.bottom_temp = temps[1]  # Второе значение - нижний нагреватель
                self.preheat_temp = temps[2] # Третье значение - преднагреватель

    # Getters для температуры каждого нагревателя
    def get_top_temp(self):
        """Возвращает температуру верхнего нагревателя"""
        return self.top_temp

    def get_bottom_temp(self):
        """Возвращает температуру нижнего нагревателя"""
        return self.bottom_temp

    def get_preheat_temp(self):
        """Возвращает температуру преднагревателя"""
        return self.preheat_temp
