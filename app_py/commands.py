class Commands:
    def __init__(self, connection):
        self.connection = connection

    def start_soldering(self, profile):
        if profile == 'Профиль 1':
            self.connection.send('A160')
            self.connection.send('B170')
            self.connection.send('P180')
        elif profile == 'Профиль 2':
            self.connection.send('A170')
            self.connection.send('B180')
            self.connection.send('P190')
        elif profile == 'Профиль 3':
            self.connection.send('A180')
            self.connection.send('B190')
            self.connection.send('P200')

    def stop_soldering(self):
        self.connection.send('T')
        self.connection.send('U')
        self.connection.send('H')

    def get_temperature_data(self):
        self.connection.send('GET_TEMP')
        return self.connection.receive()
