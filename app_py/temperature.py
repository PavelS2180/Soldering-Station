class Temperature:
    def __init__(self, connection):
        self.connection = connection
        self.top_temp = 0
        self.bottom_temp = 0
        self.preheat_temp = 0

    def update_temperatures(self):
        data = self.connection.receive()
        if data:
            temps = data.split(',')
            if len(temps) == 3:
                self.top_temp = temps[0]
                self.bottom_temp = temps[1]
                self.preheat_temp = temps[2]

    def get_top_temp(self):
        return self.top_temp

    def get_bottom_temp(self):
        return self.bottom_temp

    def get_preheat_temp(self):
        return self.preheat_temp
