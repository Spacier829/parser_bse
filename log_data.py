import time


class logData:
    filename = ""

    def __init__(self):
        self.filename = time.strftime('%Y-%m-%d_%H-%M-%S') + ".txt"

    def save_data(self, data_header, data):
        self.file = open(self.filename, 'a', encoding="utf-8")
        self.file.write(data_header)
        self.file.write(data)
        self.file.close()