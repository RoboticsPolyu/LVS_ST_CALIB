import numpy as np
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import optimize

def readFile(filepath):
    with open(filepath,"r") as f:
        while (True):
            yield f.readline().strip()

filepath = "data.txt"
#list = open(filepath,"r").readlines().strip()
time = 0
list = []

XYZ = np.zeros((5,2500))
for i in readFile(filepath):
    if (time >=2500):
        break
    li = [float(j) for j in re.split('\s',i)]
    XYZ[0][time] = li[0]
    XYZ[1][time] = li[1]
    XYZ[2][time] = li[2]
    XYZ[3][time] = li[3]
    XYZ[4][time] = li[4]
    time = time + 1
    #print (li)
print (XYZ)
fig = plt.figure()
ax = plt.subplot(111,projection='3d')
ax.scatter(XYZ[0][:2500],XYZ[1][:2500],XYZ[2][:2500],c='y')
#ax.scatter(XYZ[0][100:200],XYZ[1][100:200],XYZ[2][100:200],c='r')
#ax.scatter(XYZ[0][200:2500],XYZ[1][200:2500],XYZ[2][200:2500],c='g')
plt.show()

X = XYZ[0][0:2500]
Y = XYZ[1][0:2500]
def residuals(p):
    k,b = p
    return Y - (k*X+b)

r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]

k =  -0.02222632194863699 
b =  21.732792012553297

print("k = ", k, " b = " ,b)

YY = k* X +b
plt.plot(X, YY, 'b-')


plt.plot(XYZ[0][0:2500], YY, 'b-');
plt.plot(XYZ[0][0:2500], XYZ[1][0:2500], 'r.-');
plt.show();
plt.plot(XYZ[4][0:2500],'r.-');
plt.show();
plt.plot(XYZ[1][0:2500],'b-');
plt.show();

Z = XYZ[2][0:2500]
def residuals(p):
    k,b = p
    return Z - (k*X+b)

r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]
k =  0.02153696843595345  
b =  1095.376140289066
ZZ = k* X +b
plt.plot(X, ZZ, 'b')
plt.plot(XYZ[0][0:2500], XYZ[2][0:2500], 'ro');
plt.show();
