import serial
import numpy
import time

ser = serial.Serial('com14', 115200)
time.sleep(0.5)
ser.write(b'NR')
time.sleep(0.5)
ser.readline()
ser.write(b'P0.0;0.0R')

v1 = [0.0]
v2 = [0.0]
r1 = [0.0]
r2 = [0.0]
t = [0.0]

start_time = time.time()
while (time.time() - start_time)<5:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        data = line.split(';')
        try:
            data = [float(i) for i in data]
            v1.append(float(data[0]))
            v2.append(float(data[1]))
            r1.append(float(data[2]))
            r2.append(float(data[3]))
            t.append(time.time() - start_time)
        except:
            pass

numpy.save('v1.npy', v1)
numpy.save('v2.npy', v2)
numpy.save('r1.npy', r1)
numpy.save('r2.npy', r2)
numpy.save('t.npy', t)