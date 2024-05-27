import time

import serial
import serial.tools.list_ports
import queue
from frame_status import FrameStatus


class Communication:
    com_port = ''
    baudrate = ''
    timeout = 0
    connection = serial.Serial()
    ports = serial.tools.list_ports.comports()

    def __init__(self):
        self.baudrate = 115200
        for port in sorted(self.ports):
            print("{}".format(port))
        # self.port_name = input("Write serial port name (ex: COM1): ")
        self.port_name = "COM2"

    def connect(self):
        try:
            self.connection = serial.Serial(self.port_name, self.baudrate)
        except serial.serialutil.SerialException:
            print("Can't open: ", self.port_name)

    def com_port_is_open(self):
        return self.connection.is_open

    def close(self):
        if self.com_port_is_open():
            self.connection.close()
        else:
            print(self.port_name, " is already closed")

    def read_data(self):
        header = "c0c0"
        message = queue.Queue()
        data_frame = ""
        while self.connection.is_open:
            # while True:
            time.sleep(0.1)
            read_frame_hex = self.connection.read_all().hex()
            # print(read_frame_hex)
            # read_frame_hex = "effdffffbc09e7034061c0c0d116000084c4fbff7ea6080059800100acf8ffff73feffff9a08e80320bcc0c0f81400001b69fbfff4020a009880010076f8ffffa0feffff9a08e903205bc0c0261100000e70fbff92950a002180010047f8ffffdefeffff9a08ea038a0cc0c09d0b0000a175fbffcaae0a00b57f010024f8ffff2effffffbc09eb03d0e8c0c07beaffff70f80300c134f7ffc37e010064f7ffffcbffffff9a0856040021c0c03fddffffe25304003646f6ff2a7f010054f7ffffda"
            while len(read_frame_hex) > 0:
                start_index = read_frame_hex.find(header)
                if start_index != -1:
                    if len(data_frame) == 0:
                        end_index = read_frame_hex.find(header,
                                                        start_index +
                                                        len(header) - 1)
                        if end_index != -1:
                            for i in range(start_index, end_index):
                                data_frame += read_frame_hex[i]
                            # read_frame_info(data_frame)
                            print("DATA FRAME")
                            print(data_frame)
                            data_frame = ""
                            read_frame_hex = read_frame_hex[end_index:]
                        else:
                            data_frame = read_frame_hex
                            read_frame_hex = ""
                    else:
                        end_index = start_index
                        for i in range(end_index):
                            data_frame += read_frame_hex[i]
                        # read_frame_info(data_frame)
                        print("DATA FRAME")
                        print(data_frame)
                        data_frame = ""
                        read_frame_hex = read_frame_hex[end_index:]
                else:
                    read_frame_hex = ""
                    data_frame = ""

    def get_frame_status(self, frame):
        if len(frame) != 64:
            return FrameStatus.bad
        else:
            is_good_frame = self.check_crc(frame)
            if is_good_frame:
                return FrameStatus.good
            else:
                return FrameStatus.error

    def count_frames(self, frame_status):
        self.COUNTER_ALL_FRAMES += 1
        if frame_status == FrameStatus.bad:
            self.COUNTER_BAD_FRAMES += 1
        elif frame_status == FrameStatus.good:
            self.COUNTER_GOOD_FRAMES += 1
        elif frame_status == FrameStatus.error:
            self.COUNTER_ERROR_FRAMES += 1


communication = Communication()
communication.connect()
communication.read_data()
