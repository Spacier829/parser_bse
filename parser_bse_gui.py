import serial
import queue
import time
from PyQt6 import QtCore, QtWidgets

import pyqtgraph

COUNTER_ALL_FRAMES = 0
COUNTER_GOOD_FRAMES = 0
# Counter of frames with greater or longer length
COUNTER_BAD_FRAMES = 0
# Counter of frames which calculated CRC doesn't equal read CRC
COUNTER_ERROR_FRAMES = 0

filename = ""


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        



def read_data_from_com_port(com, baudrate, bytesize):
    connection = serial.Serial(com, baudrate=baudrate, bytesize=bytesize)
    header = "c0c0"
    global filename

    # Поскольку заголовок фрейма не всегда находится в начале сообщения,
    # используется 2 переменные,
    # чтобы дозаписать данные в 1-ую часть до заголовка,
    # и считать данные после заголовка во 2-ую часть
    # ...__ __ C0 C0 __ __ __...
    # ...__ __ C0 C0 __ __ __...
    # ...__ __ C0 C0 __ __ __...

    first_part = ""
    second_part = ""
    message = queue.Queue()
    filename = time.strftime('%Y-%m-%d_%H-%M-%S') + ".txt"
    while True:

        read_frame_hex = connection.read(32).hex()
        message.put(read_frame_hex)
        print("-------------------------------------------")
        print_file("\n-------------------------------------------"
                   "\nHEX DATA:",
                   '\n' + read_frame_hex)
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
            read_frame_info(frame)
            first_part = second_part
            second_part = ""


def read_frame_info(frame):
    frame_status = is_frame_good_bad_error(frame)
    count_frames(frame_status)
    print_frame_info(frame)
    print_frames_count()
    if frame_status == "good":
        print_converted_values(frame)


def is_frame_good_bad_error(frame):
    if len(frame) != 64:
        return "bad"
    else:
        is_good_frame = check_crc(frame)
        if is_good_frame:
            return "good"
        else:
            return "error"


def check_crc(frame):
    calculated_crc = crc_16(frame)
    read_crc = hex(int(frame[62] + frame[63] + frame[60] + frame[61], 16))
    if calculated_crc == read_crc:
        return True
    else:
        return False


def crc_16(data):
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


def count_frames(frame_status):
    global COUNTER_BAD_FRAMES
    global COUNTER_ALL_FRAMES
    global COUNTER_GOOD_FRAMES
    global COUNTER_ERROR_FRAMES
    COUNTER_ALL_FRAMES += 1
    if frame_status == "bad":
        COUNTER_BAD_FRAMES += 1
    elif frame_status == "good":
        COUNTER_GOOD_FRAMES += 1
    elif frame_status == "error":
        COUNTER_ERROR_FRAMES += 1


def print_frame_info(frame):
    result_frame = ""
    for i in range(len(frame)):
        result_frame += frame[i]
        if ((i + 1) % 2) == 0:
            result_frame += ' '

    print_file("\n-------------------------------------------"
               "\nFRAME:",
               '\n' + result_frame)
    print("FRAME:")
    print(result_frame)


def print_frames_count():
    print("ALL FRAMES COUNT:", COUNTER_ALL_FRAMES)
    print_file("\nALL FRAMES COUNT:", str(COUNTER_ALL_FRAMES))
    print("GOOD FRAMES COUNT:", COUNTER_GOOD_FRAMES)
    print_file("\nGOOD FRAMES COUNT:", str(COUNTER_GOOD_FRAMES))
    print("ERROR FRAMES COUNT:", COUNTER_ERROR_FRAMES)
    print_file("\nERROR FRAMES COUNT:", str(COUNTER_ERROR_FRAMES))
    print("BAD FRAMES COUNT:", COUNTER_BAD_FRAMES)
    print_file("\nBAD FRAMES COUNT:", str(COUNTER_BAD_FRAMES))


def print_converted_values(frame):
    print_angular_velocity(frame)
    print_acceleration(frame)
    print_temperature(frame)


def print_angular_velocity(frame):
    gx, gy, gz = calculate_angular_velocity(frame)
    print("Gx = ", gx, "Gy = ", gy, "Gz = ", gz)
    print_file("\nGx = ", str(gx))
    print_file(" Gy = ", str(gy))
    print_file(" Gz = ", str(gz))


def calculate_angular_velocity(frame):
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


def print_acceleration(frame):
    ax, ay, az = calculate_acceleration(frame)
    print("Ax = ", ax, "Ay = ", ay, "Az = ", az)
    print_file("\nAx = ", str(ax))
    print_file(" Ay = ", str(ay))
    print_file(" Az = ", str(az))


def calculate_acceleration(frame):
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


def calculate_temperature(frame):
    counter_frames = bin(int(frame[58] + frame[59] +
                             frame[56] + frame[57], 16))[2:]
    if len(counter_frames) < 3:
        return 404, 0
    bit_data = int(counter_frames[-3] + counter_frames[-2] +
                   counter_frames[-1], 2)
    add_telemetry = int(frame[54] + frame[55] + frame[52] +
                        frame[53], 16) * 0.01
    return bit_data, add_telemetry


def print_temperature(frame):
    bit_data, add_telemetry = calculate_temperature(frame)
    if bit_data == 0:
        print("VOG °C__X: ", add_telemetry)
        print_file("\nVOG °C__X: ", str(add_telemetry))
    elif bit_data == 1:
        print("VOG °C__Y: ", add_telemetry)
        print_file("\nVOG °C__Y: ", str(add_telemetry))
    elif bit_data == 2:
        print("VOG °C__Z: ", add_telemetry)
        print_file("\nVOG °C__Z: ", str(add_telemetry))
    elif bit_data == 3:
        print("BA °C__X: ", add_telemetry)
        print_file("\nBA °C__X: ", str(add_telemetry))
    elif bit_data == 4:
        print("BA °C__Y: ", add_telemetry)
        print_file("\nBA °C__Y: ", str(add_telemetry))
    elif bit_data == 5:
        print("BA °C__Z: ", add_telemetry)
        print_file("\nBA °C__Z: ", str(add_telemetry))
    elif bit_data == 404:
        print("Temperature not read")
        print_file("\nTemperature not read ", str(add_telemetry))


def print_file(print_header, data):
    txt_file = open(filename, 'a', encoding="utf-8")
    txt_file.write(print_header)
    txt_file.write(data)
    txt_file.close()


read_data_from_com_port("COM2", 115200, 8)