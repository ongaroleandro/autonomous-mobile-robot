import csv
import numpy as np
from matplotlib import pyplot as plt
import scipy.stats as stats
import pandas as pd

def read_data(datafile):
	data = pd.read_csv(datafile)

	return np.array(data['x']), np.array(data['y'])

def plotdata(data, color):
	for x in data:
		plt.plot(x[0], x[1], color)

def plotGaussian(mu, var, sigma, label, subplot):
	x = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
	ax[subplot].plot(x, stats.norm.pdf(x, mu, sigma), label=label)
	ax[subplot].axvline(mu, ls='--', lw=0.5, c='gray')

def calcGaussian(array):
	mu = np.mean(array, axis=0)
	var = np.var(array, axis=0)
	sigma = np.sqrt(var)

	return mu, var, sigma


fig, ax = plt.subplots(2, 1)

dataFiles = ['data_38400.csv', 'data_57600.csv', 'data_115200.csv']
pandas = []
for file in dataFiles:
	label = file[5:-4]
	x_values, y_values = read_data(file)
	
	mu_x, var_x, sigma_x = calcGaussian(x_values)
	plotGaussian(mu_x, var_x, sigma_x, label, 0)

	mu_y, var_y, sigma_y = calcGaussian(y_values)
	plotGaussian(mu_y, var_y, sigma_y, label, 1)

	pandas.append([label, mu_x, var_x, sigma_x, mu_y, var_y, sigma_y])

df = pd.DataFrame(pandas, columns=['Baudrate', 'mu_x', 'var_x', 'sigma_x', 'mu_y', 'var_y', 'sigma_y'])
print(df)


ax[0].set_ylim(bottom=0)
ax[1].set_ylim(bottom=0)
ax[0].set_title('x-direction')
ax[1].set_title('y-direction')
ax[0].legend(loc="upper right")
ax[1].legend(loc="upper right")
plt.show()