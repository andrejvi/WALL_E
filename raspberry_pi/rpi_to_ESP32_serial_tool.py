import struct
import serial
import time
import os

""" Pythonskript for å teste seriekommunikasjon mellom RPi (eller pc) og ESP32/Arduino
            prosjektarbeid gruppe 22, IELET1002 vår 2022 """


if os.name == "posix":
    PORT = "/dev/ttyUSB0"
else:
    PORT = "COM"

BAUD = 115200

ser = serial.Serial(PORT, BAUD)
pkg_datatypes = "<BBBBBBHHeBBBB"    # "B": uint8_t, "H": uint16_t, "e": 16float, "<": little endian encoded


def get_packed_data(state=0, Kp=0, Ki=0, Kd=0, battery=0, speed=0, distance=0, u_state=0, u_Kp=0, u_Ki=0, u_Kd=0):
    start_byte = ord("<")
    stop_byte = ord(">")
    data_package = struct.pack(pkg_datatypes, start_byte, state, Kp, Ki, Kd, battery, speed, distance, u_state, u_Kp, u_Ki, u_Kd, stop_byte)
    return data_package


def input_state_request():
    print("Skriv 'quit' for å avslutte")
    while True:
        requested_state = input("Req. state (num): ")
        if requested_state.isdigit():
            requested_state = int(requested_state)
            if requested_state <= 255:
                data_package = get_packed_data(state=requested_state, u_state=1)
                print(data_package)
                ser.write(data_package)
        elif requested_state == "quit":
            break
        else:
            print("Must be int: 0--255")


def stress_test():
    delay_time = int(input("delaytid [ms]: "))
    for i in range(1000):
        for i in range(0, 12):
            data_package = get_packed_data(state=i, u_state=1)
            ser.write(data_package)
            time.sleep(delay_time/1000)

def main():
    while True:
        print(" SERIAL ----> ESP32 --(esp now)----> ESP32 ")
        print(f" baud: {BAUD}")
        print(f" port: {PORT}")
        print("[1] forespør tilstandsendring")
        print("[2] stresstesting")
        
        choice = int(input("velg funksjon: "))
        if choice == 1:
            input_state_request()
        if choice == 2:
            stress_test()

main()
