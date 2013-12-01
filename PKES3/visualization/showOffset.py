import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from numpy import genfromtxt

num_bins = 150
range_limits = [100,400]

def readFile(name):
  data_file = genfromtxt(name, delimiter='|')
  offset_data=data_file[:,0]
  return offset_data

offset_data_raw = readFile('filtered_256.data')
mu = np.mean(offset_data_raw)
sigma=np.std(offset_data_raw)
n, bins, patches = plt.hist(offset_data_raw, num_bins, range=range_limits, normed=1, facecolor='green', alpha=0.5)
plt.text(110,0.05,'MPU6050_DLPF_BW_256\n  $\mu=%3.1f$, $\sigma=%3.1f$'%(mu,sigma))
y = mlab.normpdf(bins, mu, sigma)
plt.plot(bins, y, 'r--')

offset_data_42 = readFile('filtered_42.data')
mu = np.mean(offset_data_42)
sigma=np.std(offset_data_42)
n, bins, patches = plt.hist(offset_data_42, num_bins, range=range_limits, normed=1, facecolor='blue', alpha=0.5)
plt.text(110,0.08,'MPU6050_DLPF_BW_42\n  $\mu=%3.1f$, $\sigma=%3.1f$'%(mu,sigma))
y = mlab.normpdf(bins, mu, sigma)
plt.plot(bins, y, 'r--')

offset_data_5 = readFile('filtered_5.data')
mu = np.mean(offset_data_5)
sigma=np.std(offset_data_5)
n, bins, patches = plt.hist(offset_data_5, num_bins, range=range_limits, normed=1, facecolor='red', alpha=0.3)
plt.text(110,0.12,'MPU6050_DLPF_BW_5\n $\mu=%3.1f$, $\sigma=%3.1f$'%(mu,sigma))
y = mlab.normpdf(bins, mu, sigma)
plt.plot(bins, y, 'r--')

plt.xlabel('Gyro Offsets')
plt.ylabel('Probability')
plt.title('Histogram of MPU 6050 Gyro Offsets')

# Tweak spacing to prevent clipping of ylabel
plt.subplots_adjust(left=0.15)
plt.show()