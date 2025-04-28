import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

import control as ct

data :dict = np.load("data_test_161224_PWM70-70_T4.npy", allow_pickle=True).item()

t = data['time']
vl = data["vl"]
va = data["va"]
print(vl)
for i in range(len(vl)):
    vl[i] = float(vl[i])
    va[i] = -float(va[i])


#plt.figure()
#plt.plot(t, vl)

def second_order_model(t, omega_n, zeta, K):
    # Calcul de la réponse à un échelon
    phi = np.arccos(zeta)
    return K * (1 - (1 / np.sqrt(1 - zeta**2)) * np.exp(-zeta * omega_n * t) *
                np.sin(omega_n * np.sqrt(1 - zeta**2) * t + phi))


# Ajustement des données
initial_guess = [7, 0.5, 1]  # omega_n, zeta, K (valeurs initiales à ajuster si nécessaire)
popt, _ = curve_fit(second_order_model, t, vl, p0=initial_guess)
omega_n, zeta, K = popt

# Afficher les résultats ajustés
print(f"Paramètres ajustés :")
print(f"  ω_n = {omega_n:.2f}")
print(f"  ζ   = {zeta:.2f}")
print(f"  K   = {K:.2f}")

# Tracé des résultats
time_fit = np.linspace(min(t), max(t), 1000)
omega_n = 1/0.45
zeta = 0.5
K = 2.1/150

p = ct.TransferFunction([K], [1/omega_n**2, 2*zeta/omega_n, 1])
va_fit = ct.forced_response(p, T=time_fit, U=150)[1]

plt.figure()
plt.plot(t, va, 'o', label="Données expérimentales")
plt.plot(time_fit, va_fit, '-', label=f"Modèle ajusté (ω_n={omega_n:.2f}, ζ={zeta:.2f}, K={K:.2f})")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse (m/s)")
plt.title("Ajustement d'un modèle du second ordre")
plt.legend()
plt.grid()
plt.show()




plt.show()