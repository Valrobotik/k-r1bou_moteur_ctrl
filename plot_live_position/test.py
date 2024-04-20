import numpy as np

import matplotlib.pyplot as plt

x1, y1 = 2, 0
x2, y2 = 1, 0

center = [np.mean([x1, x2]), np.mean([y1, y2])] #center of the line

vectors = np.array([[x1, y1], [x2, y2]]) - center #vectors from the center to the corners
print(vectors)
angles = np.arctan2(vectors.T[:, 1], vectors.T[:, 0]) #angles of the vectors

plt.plot(x1, y1, 'ro')
plt.plot(x2, y2, 'bo')

plt.quiver(*center, *vectors[0], angles='xy', scale_units='xy', scale=1, color='r')
plt.quiver(*center, *vectors[1], angles='xy', scale_units='xy', scale=1,  color='b')

a = np.mean(angles)

plt.quiver(*center, *[0.5*np.cos(a), 0.5*np.sin(a)], angles='xy', scale_units='xy', scale=1, color='g')

print(3.14/4)

plt.show()  

print(vectors)
print(angles)
print(np.mean(angles))
