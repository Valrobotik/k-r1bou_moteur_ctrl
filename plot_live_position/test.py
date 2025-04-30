import numpy as np
import matplotlib.pyplot as plt
import serial
import time

total_time_sim = 4

ser = serial.Serial("COM16", 11500)
time.sleep(0.2)
ser.write(b'QXF')

t0 = time.time()
t = [0]
vl = [0]
va = [0]

ser.write(b"BOF")
ser.write(b"VL150VR-150F")

while t[-1]<total_time_sim :
    if ser.in_waiting:
        x = ser.read_until(b"\n").decode().replace("\r\n", "").split(';')
        if len(x)==6:
            try : 
                vl.append(float(x[3]))
                va.append(float(x[4]))
                t.append(time.time()-t0)
                print(t[-1])
            except:
                pass

ser.write(b"SXF")

data = {
    "time" : t,
    "vl" : vl,
    "va" : va,
}

np.save("data_test_161224_PWM70-70_T4.npy", data)

plt.figure()
plt.plot(t, vl, "vl")
plt.legend()
plt.figure()
plt.plot(t, va, "va")
plt.legend()

plt.show()