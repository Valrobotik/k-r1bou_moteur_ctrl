import numpy
import matplotlib.pyplot as plt
import control

v1 = numpy.load('v1.npy')
v2 = numpy.load('v2.npy')
r1 = numpy.load('r1.npy')
r2 = numpy.load('r2.npy')
t = numpy.load('t.npy')

#fonction de transfert aproximative du systeme mesurer ci dessus
F = control.TransferFunction([0.37], [0.21, 1])
t_p = numpy.linspace(0, 7, 1000)
tp, y = control.forced_response(F, T=t_p, U=1)


v1p = [v1[0]]
v2p = [v2[0]]
for i in range(1, len(t)):
    coef = 0.5
    v1p.append((v1[i-1]*coef+v1[i]*(1-coef)))
    v2p.append((v2[i-1]*coef+v2[i]*(1-coef)))
    t[i] = t[i] - 1

for i in range(0, len(r1)):
    r1[i] = r1[i]*0.1
    r2[i] = r2[i]*0.1

fig1 = plt.figure()
plt.plot(t, v1p, label='vitesse lineaire droite (m/sec)')
plt.plot(t, r1, label='position angulaire droite (*0.1rad)')
plt.plot(tp, y, label='aprox premier ordre')
#plt.plot(t, v2p, label='v2')
plt.xlim(0, 4)
plt.ylim(-0.5, 1)
plt.legend()
fig1.show()
fig2 = plt.figure()
plt.plot(t, v2p, label='vitesse lineaire gauche (m/sec)')
plt.plot(t, r2, label='position angulaire gauche (*0.1rad)')
plt.plot(tp, y, label='aprox premier ordre')

plt.xlim(0, 4)
plt.ylim(-0.5, 1)
plt.legend()
fig2.show()
input()

p = control.TransferFunction([1], [1, 0])
t, y = control.forced_response(p, T=t, U=v1p)

