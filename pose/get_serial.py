import serial, csv, os
if os.path.isfile('output.csv'): os.remove('output.csv')
arduino = serial.Serial(port='COM8', baudrate=115200, timeout=.1)
with open('output.csv','w') as csvfile:
    w = csv.writer(csvfile, delimiter=',')
    try:
        while True:
            line = arduino.readline().decode().rstrip().split()
            print(line)
            if len(line) == 7:
                line = list(map(float, line))
                w.writerow(line)
    finally:
        csvfile.close()
        print("File closed!")