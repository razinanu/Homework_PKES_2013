import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from numpy import genfromtxt

from scipy import fft, arange

num_bins = 7
range_limits = [27,33]

def readFile(name):
  data_file = genfromtxt(name, delimiter='\t')
  gyro_data=data_file[:,0:2]
  return gyro_data

gyro_data = readFile('delay.data')

gyro_data_raw = np.diff(gyro_data[:,0])
n, bins, patches = plt.hist(gyro_data_raw, num_bins, normed=1, facecolor='green', alpha=0.5)
mu = np.mean(gyro_data_raw)
sigma=np.std(gyro_data_raw)
plt.text(28,0.3,'$\mu=%3.1f$, $\sigma=%3.1f$'%(mu,sigma))
plt.xlabel('Milliseconds')
plt.ylabel('Probability')
plt.title('Delay of one MPU 6050 Gyro loop')

plt.subplots_adjust(left=0.15)
plt.show()
print 'Aus Maus'