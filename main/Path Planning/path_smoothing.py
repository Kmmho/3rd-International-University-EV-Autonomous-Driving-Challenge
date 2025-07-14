import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
import rospkg
import os
import csv


EASTOFFSET = 313008.55819800857
NORTHOFFSET = 4161698.628368007

rospack = rospkg.RosPack()
ref_path = []
filename = os.path.join(rospack.get_path('dku_morai_driver'),
                                             f'ref_path/bonsun.csv')


with open(filename, newline='') as csvfile:
    path = csv.reader(csvfile, delimiter=',', quotechar='|')
    for i, row in enumerate(path):
        if i % 6 == 0:
            ref_path.append([float(row[0])-EASTOFFSET, float(row[1])-NORTHOFFSET])

    if ref_path[-1] != [float(row[0])-EASTOFFSET, float(row[1])-NORTHOFFSET]:
        ref_path.append([float(row[0])-EASTOFFSET, float(row[1])-NORTHOFFSET])

path_array = np.array(ref_path)

x_p = path_array[:, 0]
y_p = path_array[:, 1]

dx = np.diff(path_array[:, 0])
pre_dx = dx[0]
idx = []
for i in range(1, len(dx)):
    cur_dx = dx[i]
    if cur_dx*pre_dx < 0:
        idx.append(i)
    pre_dx = cur_dx

i = 0
for k in idx:
    path_array = np.delete(path_array, k-i, axis=0)
    i += 1
    
x = path_array[:, 0]
y = path_array[:, 1]
    
l = len(path_array)
Order = 2

# t = np.linspace(0, 1, max(l*2, 70), endpoint=True)
t = np.linspace(0, 1, l-(Order-1), endpoint=True)
t = np.append(np.zeros(Order), t)
t = np.append(t, np.ones(Order))

tck = [t, [x,y], Order]
u3 = np.linspace(0, 1, max(8*l, 70), endpoint=True)
out = interpolate.splev(u3, tck)

path = []
for j in range(len(out[0])):
    path.append([out[0][j], out[1][j]])

plt.plot(x_p ,y_p, '--')
plt.plot(x, y, 'r')
plt.plot(out[0], out[1], 'b')
for k in idx:
    plt.plot(x_p[k], y_p[k], 'ro')
plt.show()

fn = 'bonsun_smoothing2'
filename = os.path.join(rospack.get_path('dku_morai_driver'), 
                                                 f'ref_path/{fn}.csv')

with open(filename, 'w', newline='') as csvfile:
    path_writer = csv.writer(csvfile)
    for row in path:
        path_writer.writerow(row)
