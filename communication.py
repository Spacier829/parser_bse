import serial
import queue
import serial.tools.list_ports
from log_data import logData
from graphs.test_graph import MainWindow


class Communication:
    COUNTER_ALL_FRAMES = 0
    COUNTER_GOOD_FRAMES = 0
    # Counter of frames with greater or longer length
    COUNTER_BAD_FRAMES = 0
    # Counter of frames which calculated CRC doesn't equal read CRC
    COUNTER_ERROR_FRAMES = 0

    com_port = ''
    baudrate = ''
    ports = serial.tools.list_ports.comports()
    connection = serial.Serial()
    log = logData()
    plot = MainWindow()

    def __init__(self):
        self.baudrate = 115200
        print("Available ports are: ")
        for port in sorted(self.ports):
            print("{}".format(port))
        self.port_name = input("Write serial port name (ex: COM1): ")
        try:
            self.connection = serial.Serial(self.port_name, self.baudrate)
            self.log.save_data("Serial port : ", self.port_name)
        except serial.serialutil.SerialException:
            print("Can't open : ", self.port_name)

    def close(self):
        if self.connection.is_open:
            self.connection.close()
        else:
            print(self.port_name, " is already closed")

    def read_data(self):
        header = "c0c0"
        first_part = ""
        second_part = ""
        message = queue.Queue()
        while self.connection.is_open:

            read_frame_hex = self.connection.read(32).hex()
            message.put(read_frame_hex)
            self.log.save_data(
                "\n-------------------------------------------"
                "\nHEX DATA:",
                '\n' + read_frame_hex)
            print("-------------------------------------------")
            print("HEX DATA:")
            print(read_frame_hex)

            if message.qsize() == 2:
                data = message.get()
                index = data.find(header)
                if index != -1:
                    if len(first_part) == 0:
                        for i in range(index, len(data)):
                            first_part += data[i]
                    else:
                        if len(second_part) == 0:
                            for i in range(index, len(data)):
                                second_part += data[i]
                        for i in range(index):
                            first_part += data[i]

            if len(second_part) != 0:
                frame = first_part
                self.read_frame_info(frame)
                first_part = second_part
                second_part = ""

    def read_frame_info(self, frame):
        frame_status = self.is_frame_good_bad_error(frame)
        self.count_frames(frame_status)
        self.print_frame_info(frame)
        self.print_frames_count()
        if frame_status == "good":
            self.print_converted_values(frame)

    def is_frame_good_bad_error(self, frame):
        if len(frame) != 64:
            return "bad"
        else:
            is_good_frame = self.check_crc(frame)
            if is_good_frame:
                return "good"
            else:
                return "error"

    def check_crc(self, frame):
        calculated_crc = self.crc_16(frame)
        read_crc = hex(int(frame[62] + frame[63] + frame[60] + frame[61], 16))
        if calculated_crc == read_crc:
            return True
        else:
            return False

    def crc_16(self, data):
        polynomial = 0x1021
        test = ""
        for i in range(4, len(data) - 4):
            test += data[i]

        data = bytearray.fromhex(test)

        crc = 0xFFFF

        for c in data:
            crc ^= (c << 8)
            for j in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc = crc << 1

        crc = crc & 0xFFFF
        crc = hex(crc)
        return crc

    def count_frames(self, frame_status):

        self.COUNTER_ALL_FRAMES += 1
        if frame_status == "bad":
            self.COUNTER_BAD_FRAMES += 1
        elif frame_status == "good":
            self.COUNTER_GOOD_FRAMES += 1
        elif frame_status == "error":
            self.COUNTER_ERROR_FRAMES += 1

    def print_frame_info(self, frame):
        result_frame = ""
        for i in range(len(frame)):
            result_frame += frame[i]
            if ((i + 1) % 2) == 0:
                result_frame += ' '

        self.log.save_data("\n-------------------------------------------"
                           "\nFRAME:",
                           '\n' + result_frame)
        print("FRAME:")
        print(result_frame)

    def print_frames_count(self):
        print("ALL FRAMES COUNT:", self.COUNTER_ALL_FRAMES)
        self.log.save_data("\nALL FRAMES COUNT:",
                           str(self.COUNTER_ALL_FRAMES))
        print("GOOD FRAMES COUNT:", self.COUNTER_GOOD_FRAMES)
        self.log.save_data("\nGOOD FRAMES COUNT:",
                           str(self.COUNTER_GOOD_FRAMES))
        print("ERROR FRAMES COUNT:", self.COUNTER_ERROR_FRAMES)
        self.log.save_data("\nERROR FRAMES COUNT:",
                           str(self.COUNTER_ERROR_FRAMES))
        print("BAD FRAMES COUNT:", self.COUNTER_BAD_FRAMES)
        self.log.save_data("\nBAD FRAMES COUNT:",
                           str(self.COUNTER_BAD_FRAMES))

    def print_converted_values(self, frame):
        self.print_angular_velocity(frame)
        self.print_acceleration(frame)
        self.print_temperature(frame)

    def print_angular_velocity(self, frame):
        gx, gy, gz = self.calculate_angular_velocity(frame)
        print("Gx = ", gx, "Gy = ", gy, "Gz = ", gz)
        self.log.save_data("\nGx = ", str(gx))
        self.log.save_data(" Gy = ", str(gy))
        self.log.save_data(" Gz = ", str(gz))

    def calculate_angular_velocity(self, frame):
        gx = (frame[10] + frame[11] + frame[8] + frame[9] +
              frame[6] + frame[7] + frame[4] + frame[5])
        gx = int(gx, 16) * (1.085069 * (10 ** (-6)))

        gy = (frame[18] + frame[19] + frame[16] + frame[17] +
              frame[14] + frame[15] + frame[12] + frame[13])
        gy = int(gy, 16) * (1.085069 * (10 ** (-6)))

        gz = (frame[26] + frame[27] + frame[24] + frame[25] +
              frame[22] + frame[23] + frame[20] + frame[21])
        gz = int(gz, 16) * (1.085069 * (10 ** (-6)))

        return gx, gy, gz

    def print_acceleration(self, frame):
        ax, ay, az = self.calculate_acceleration(frame)
        print("Ax = ", ax, "Ay = ", ay, "Az = ", az)
        self.log.save_data("\nAx = ", str(ax))
        self.log.save_data(" Ay = ", str(ay))
        self.log.save_data(" Az = ", str(az))

    def calculate_acceleration(self, frame):
        # ax = (frame[30] + frame[31] + frame[28] + frame[29] +
        #       frame[34] + frame[35] + frame[32] + frame[33])
        # ax = int(ax, 16) * 1.0 * (10 ** (-4))
        ax = (frame[34] + frame[35] + frame[32] + frame[33] +
              frame[30] + frame[31] + frame[28] + frame[29])
        ax = int(ax, 16) * 1.0 * (10 ** (-4))

        # ay = (frame[38] + frame[39] + frame[36] + frame[37] +
        #       frame[42] + frame[43] + frame[40] + frame[41])
        # ay = int(ay, 16) * 1.0 * (10 ** (-4))
        ay = (frame[42] + frame[43] + frame[40] + frame[41] +
              frame[38] + frame[39] + frame[36] + frame[37])
        ay = int(ay, 16) * 1.0 * (10 ** (-4))

        # az = (frame[46] + frame[47] + frame[44] + frame[45] +
        #       frame[50] + frame[51] + frame[48] + frame[49])
        # az = int(az, 16) * 1.0 * (10 ** (-4))
        az = (frame[50] + frame[51] + frame[48] + frame[49] +
              frame[46] + frame[47] + frame[44] + frame[45])
        az = int(az, 16) * 1.0 * (10 ** (-4))

        return ax, ay, az

    def calculate_temperature(self, frame):
        counter_frames = bin(int(frame[58] + frame[59] +
                                 frame[56] + frame[57], 16))[2:]
        if len(counter_frames) < 3:
            return 404, 0
        bit_data = int(counter_frames[-3] + counter_frames[-2] +
                       counter_frames[-1], 2)
        add_telemetry = int(frame[54] + frame[55] + frame[52] +
                            frame[53], 16) * 0.01
        return bit_data, add_telemetry

    def print_temperature(self, frame):
        bit_data, add_telemetry = self.calculate_temperature(frame)
        if bit_data == 0:
            print("VOG °C__X: ", add_telemetry)
            self.log.save_data("\nVOG °C__X: ", str(add_telemetry))
        elif bit_data == 1:
            print("VOG °C__Y: ", add_telemetry)
            self.log.save_data("\nVOG °C__Y: ", str(add_telemetry))
        elif bit_data == 2:
            print("VOG °C__Z: ", add_telemetry)
            self.log.save_data("\nVOG °C__Z: ", str(add_telemetry))
        elif bit_data == 3:
            print("BA °C__X: ", add_telemetry)
            self.log.save_data("\nBA °C__X: ", str(add_telemetry))
        elif bit_data == 4:
            print("BA °C__Y: ", add_telemetry)
            self.log.save_data("\nBA °C__Y: ", str(add_telemetry))
        elif bit_data == 5:
            print("BA °C__Z: ", add_telemetry)
            self.log.save_data("\nBA °C__Z: ", str(add_telemetry))
        elif bit_data == 404:
            print("Temperature not read")
            self.log.save_data("\nTemperature not read ", str(add_telemetry))
