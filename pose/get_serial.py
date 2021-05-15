import serial, csv, os
from time import sleep
if os.path.isfile('output.csv'): os.remove('output.csv')
arduino = serial.Serial(port='COM8', baudrate=115200, timeout=.1)
writing = False
f = open('output.csv', "x")
with open('output.csv','w') as csvfile:
    w = csv.writer(csvfile, delimiter=',')
    try:
        while True:
            try:
                line = arduino.readline().decode().rstrip().split()
                print(line)
            except UnicodeDecodeError:  continue
            except KeyboardInterrupt:   break

            if 'VALIDATED!' in line:
                writing = True
                continue
            
            if writing:
                line = list(map(float, line))
                line[4] += line[1]
                line[5] += line[2]
                line[6] += line[3]
                w.writerow(line)
    
    finally:
        csvfile.close()
        print("File closed!")