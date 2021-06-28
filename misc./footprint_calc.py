import numpy as np 
from matplotlib import pyplot as plt

R = 0.15

fig = plt.figure()
ax = plt.gca()

x_circle = np.linspace(-0.15, 0.15, 1000)
y_circle = -np.sqrt(R**2 - x_circle**2)

plt.plot(x_circle, y_circle, 0.1)
plt.plot([-0.15, 0.15],[0.05, 0.05], '#0072BD')
plt.plot([-0.15, -0.15], [0, 0.05], '#0072BD')
plt.plot([0.15, 0.15], [0, 0.05], '#0072BD')
plt.arrow(0, 0, -0.1, 0, color='red', length_includes_head=True)
plt.text(-0.109, 0, s='Y2', color='red', fontsize='large')
plt.arrow(0, 0, 0, 0.1, color='red', length_includes_head=True)
plt.text(0, 0.1, s='X2', color='red', fontsize='large')
plt.arrow(0, 0, 0, 0.08, color='blue', length_includes_head=True)
plt.text(0, 0.08, s='Y1', color='blue', fontsize='large')
plt.arrow(0, 0, 0.08, 0, color='blue', length_includes_head=True)
plt.text(0.08, 0, s='X1', color='blue', fontsize='large')
ax.grid(True)
ax.set_axisbelow(True)
#plt.axis('equal')
#plt.show()

RotMat = np.array([[0, 1], [-1, 0]])
footprint = []

i = -0.15
while True:
	if i > 0.15:
		break

	y= -np.sqrt(R**2 - i**2)

	dotp = np.dot(RotMat, np.transpose(np.array([round(i, 2), round(y, 2)])))
	footprint.append([dotp[0], dotp[1]])

	if i == -0.15:
		plt.plot(i, y, color='r', marker='s')
	elif i == 0.15:
		plt.plot(i, y, color='m', marker='^')
	else:
		plt.plot(i, y, color='black', marker='x')

	
	i += 0.01

#plt.axis('equal')
#plt.show()

plt.plot(0.15, 0.05, color='black', marker='o')
plt.plot(-0.15, 0.05, color='black', marker='o')

footprint.append([0.05, -0.15])
footprint.append([0.05, 0.15])
print(footprint)
plt.axis('equal')
plt.show()
