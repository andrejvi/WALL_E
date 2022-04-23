import struct
import serial
import time

""" Pythonskript for å teste seriekommunikasjon mellom RPi (eller pc) og ESP32/Arduino
            prosjektarbeid gruppe 22, IELET1002 vår 2022 """


PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD)

def input_state_request():
    print("Skriv 'quit' for å avslutte")
    while True:
        requested_state = input("Req. state (num): ")
        if requested_state.isdigit():
            requested_state = int(requested_state)
            if requested_state <= 255:
                data_package = struct.pack("<B", requested_state)
                ser.write(data_package)
        elif requested_state == "quit":
            break
        else:
            print("Must be int: 0--255")


def stress_test():
    delay_time = int(input("delaytid [ms]: "))
    for i in range(1000):
        for i in range(0, 12):
            data_package = struct.pack("<B", i)
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
