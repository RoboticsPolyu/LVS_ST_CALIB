import numpy as np
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import optimize

def readFile(filepath):
    with open(filepath,"r") as f:
        while (True):
            yield f.readline().strip()

filepath = "data_line.txt"
#list = open(filepath,"r").readlines().strip()
time = 0
list = []

XYZ = np.zeros((4,50))
for i in readFile(filepath):
    if (time >=50):
        break
    li = [float(j) for j in re.split('\s',i)]
    XYZ[0][time] = li[0]
    XYZ[1][time] = li[1]
    XYZ[2][time] = li[2]
    XYZ[3][time] = li[3]
    time = time + 1
    #print (li)
print (XYZ)
fig = plt.figure()
ax = plt.subplot(111,projection='3d')
ax.scatter(XYZ[0][:50],XYZ[1][:50],XYZ[2][:50],c='y')
#ax.scatter(XYZ[0][100:200],XYZ[1][100:200],XYZ[2][100:200],c='r')
#ax.scatter(XYZ[0][200:400],XYZ[1][200:400],XYZ[2][200:400],c='g')
plt.show()

X = XYZ[0][:50]
Y = XYZ[1][:50]

def residuals(p):
    k,b = p
    return Y - (k*X+b)

r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]
print("k = ", k, " b = " ,b)

YY = k* X +b
plt.plot(X, YY, 'b-')

plt.plot(XYZ[0][:50], XYZ[1][:50], 'ro-');
plt.show();

Z = XYZ[2][:50]

def residuals(p):
    k,b = p
    return Z - (k*X+b)
r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]
print("k = ", k, " b = " ,b)

ZZ = k* X +b
plt.plot(X, ZZ, 'b-');
plt.plot(XYZ[0][:50], XYZ[2][:50], 'ro-');
plt.show();
   
