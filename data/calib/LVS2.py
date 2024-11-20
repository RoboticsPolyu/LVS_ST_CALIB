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

XYZ = np.zeros((5,270))
for i in readFile(filepath):
    if (time >=270):
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
ax.scatter(XYZ[0][:270],XYZ[1][:270],XYZ[2][:270],c='y')
#ax.scatter(XYZ[0][100:270],XYZ[1][100:270],XYZ[2][100:270],c='r')
#ax.scatter(XYZ[0][270:270],XYZ[1][270:270],XYZ[2][270:270],c='g')
plt.show()

X = XYZ[0][0:270]
Y = XYZ[1][0:270]
def residuals(p):
    k,b = p
    return Y - (k*X+b)

r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]

#k =  -0.02222632194863699 
#b =  21.732792012553297

print("k = ", k, " b = " ,b)

YY = k* X +b


# plt.plot(XYZ[0][0:270], XYZ[1][0:270], 'b.');

start = 0;

for i in range(2,270,1):
	if XYZ[0][i-1] - XYZ[0][i] > 10:
		plt.plot(XYZ[0][start:i-1], XYZ[1][start:i-1], 'b-');
		start = i;
	
plt.plot(XYZ[0][0:270], YY, 'r-');
plt.show();

plt.plot(XYZ[4][0:270],'r-');
plt.show();


Z = XYZ[2][0:270]
def residuals(p):
    k,b = p
    return Z - (k*X+b)

r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]

ZZ = k* X +b
print("k = ", k, " b = " ,b)

plt.plot(X, Z,'b.');
plt.plot(X, ZZ, 'r-')
plt.show();

