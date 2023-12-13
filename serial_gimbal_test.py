import serial
import time

Serial = serial.Serial(port='COM9',  baudrate=115200, timeout=.1)

def write_read(x):
    # arduino.write(bytes(x,  'utf-8'))
    Serial.write(x)
    time.sleep(0.05)
    data = Serial.readline()
    return  data

if __name__ == "__main__":
    while True:
        num = input("Enter a number: ")
        thestring = "FF 01 0F 10 00 01 01 00 00 00 00 0A 00 00 00 0A 00 00 00 16"
        thestring = bytes.fromhex(thestring)
        value  = write_read(thestring)
        print(value)



