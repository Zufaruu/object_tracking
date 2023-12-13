import serial
import time

Serial = serial.Serial(port='COM10',  baudrate=115200, timeout=.1)

# def write_read(x):
#     # arduino.write(bytes(x,  'utf-8'))
#     Serial.write(x)
#     time.sleep(0.05)
#     data = Serial.readline()
#     return data

def control_parser():
    '''direction parsing
       0 - 32767 bawah kiri
       65535 - 32768 atas kanan
    '''

    Header = "FF 01 0F 10"
    RM = " 00" # Roll mode | 00 No control, 01 Mode Speed, 02 Mode Speed Angle
    PM = " 01" # Pitch Mode | 00 No control, 01 Mode Speed, 02 Mode Speed Angle
    YM = " 00" # Yaw Mode | 00 No control, 01 Mode Speed, 02 Mode Speed Angle
    Rsl = " 00" # Roll speed low byte 
    Rsh = " 00" # Roll speed high byte | 0.1220740379 degree/sec | (2 byte signed, little-endian order)
    Ral = " 00" # Roll angle low byte 
    Rah = " 00" # Roll angle  high byte | 0.02197265625 degree | (2 byte signed, little-endian order)
    Psl = " EB" # Pitch speed low byte 
    Psh = " FF" # Pitch speed high byte | 0.1220740379 degree/sec | (2 byte signed, little-endian order)
    Pal = " 00" # Pitch angle low byte 
    Pah = " 00" # Pitch angle  high byte | 0.02197265625 degree | (2 byte signed, little-endian order)
    Ysl = " 00" # Yaw speed low byte 
    Ysh = " 00" # Yaw speed high byte | 0.1220740379 degree/sec | (2 byte signed, little-endian order)
    Yal = " 00" # Yaw angle low byte 
    Yah = " 00" # Yaw angle  high byte | 0.02197265625 degree | (2 byte signed, little-endian order)
    CS =  "{0:x}".format(((int(RM[-2:], 16) + int(PM[-2:], 16) + int(YM[-2:], 16) 
                           + int(Rsl[-2:], 16) + int(Rsh[-2:], 16) + int(Ral[-2:], 16) + int(Rah[-2:], 16) 
                           + int(Psl[-2:], 16) + int(Psh[-2:], 16) + int(Pal[-2:], 16) + int(Pah[-2:], 16) 
                           + int(Ysl[-2:], 16) + int(Ysh[-2:], 16) + int(Yal[-2:], 16) + int(Yah[-2:], 16)) % 256))  # checksum
    if len(CS) == 1:
        CS = " 0" + CS
    else:
        CS = " " + CS

    command = Header + RM + PM + YM + Rsl + Rsh + Ral + Rah + Psl + Psh + Pal + Pah + Ysl + Ysh + Yal + Yah + CS
    
    return command


if __name__ == "__main__":
    while True:

        num = int(input("Enter a number: "))
        print(type(num))

        if num == 0:
            thestring = "FF 01 0F 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
        else:
            thestring = control_parser()       # checking negative direction
            # thestring = "FF 01 0F 10 00 01 01 00 00 00 00 0A 00 00 00 0A 00 00 00 16"     # speed
            # thestring = "FF 01 0F 10 00 05 05 00 00 00 00 00 00 1C 07 00 00 E4 F8 09"     # angle

        
        # print(thestring)
        # print(control_parser())
        # print(bytes.fromhex(thestring))
        # print(bytes.fromhex(control_parser()))

        thestring = bytes.fromhex(thestring)
        Serial.write(thestring)

        # value  = write_read(thestring)
        # print(value)



