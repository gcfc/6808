import serial
arduino = serial.Serial(port='COM8', baudrate=115200, timeout=.1)
while True:
    line = arduino.readline().decode().rstrip()
    print(line)