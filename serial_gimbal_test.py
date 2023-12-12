import serial

# testing windows + arduino
ser = serial.Serial(
    port='COM9',
    baudrate=115200,
)

# ser = serial.Serial(
#     port='/dev/ttyUSB0',
#     baudrate=19200,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     bytesize=serial.EIGHTBITS
# )

print(ser.isOpen())
thestring = "FF 01 0F 10 00 01 01 00 00 00 00 0A 00 00 00 0A 00 00 00 16"
thestring = bytes.fromhex(thestring)
print(thestring)

# ser.write(thestring)
ser.write(thestring)

ser.close()


