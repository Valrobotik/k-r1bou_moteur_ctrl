import serial
import numpy as np
import time
from scipy import signal

from matplotlib import pyplot as plt

ser = serial.Serial('com16', 115200)
time.sleep(0.05)
ser.write(b'QXF')
time.sleep(0.05)
ser.readline()
ser.write(b'BOF')

start_time = time.time()
end_time = 5 # 4s
Te = 0.025 # 25ms

t = np.linspace(0, end_time)

t = np.arange(0, end_time, Te) 

def trapzoid_signal(t, width=2., slope=1., amp=1., offs=0):
    a = slope*width*signal.sawtooth(2*np.pi*t/width, width=0.5)/4.
    a[a>amp/2.] = amp/2.
    a[a<-amp/2.] = -amp/2.
    return a + amp/2. + offs

comande = trapzoid_signal(t, width=end_time, slope=100, amp=50, offs=2.5)

x = []
y = []
theta = []
linear_speed = []
angular_speed = []

previous_time = start_time
i = 0

t_recup = []
while (time.time() - start_time)<end_time:
    if previous_time+Te < time.time():
        previous_time = time.time()
        if comande[i] < 5: comande[i] = 0
        ser.write(f"VL{comande[i]:.0f}VR{comande[i]:.0f}F".encode())
        print(f"VL{comande[i]:.0f}VR{comande[i]:.0f}F")    
        i+=1
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        data = line.split(';')
        try:
            data = [float(i) for i in data]
            t_recup.append(time.time()-start_time)
            x.append(data[0])
            y.append(data[1])
            theta.append(data[2])
            linear_speed.append(data[3])
            angular_speed.append(data[4])
        except:
            pass
    ser.write(f"SXF".encode())

# while (time.time() - start_time)<end_time+3:
#     if ser.in_waiting > 0:
#         line = ser.readline().decode('utf-8').strip()
#         data = line.split(';')
#         try:
#             data = [float(i) for i in data]
#             t_recup.append(time.time()-start_time)
#             x.append(data[0])
#             y.append(data[1])
#             theta.append(data[2])
#             linear_speed.append(data[3])
#             angular_speed.append(data[4])
#         except:
#             pass


plt.figure()
plt.plot(t, comande, label='comande')
plt.legend()

plt.figure()
plt.plot(t_recup, x, label='x')
plt.plot(t_recup, y, label='y')
plt.legend()

plt.figure()
plt.plot(t_recup, theta, label='theta')
plt.legend()

plt.figure()
plt.plot(t_recup, linear_speed, label='linear_speed')
plt.plot(t_recup, angular_speed, label='angular_speed')
plt.plot(t, comande/100, label='comande')
plt.legend()
plt.show()

data = {
    't': t_recup,
    'x': x,
    'y': y,
    'theta': theta,
    'linear_speed': linear_speed,
    'angular_speed': angular_speed,
    't': t,
    'comande': comande
}

np.save('data_3.npy', data)