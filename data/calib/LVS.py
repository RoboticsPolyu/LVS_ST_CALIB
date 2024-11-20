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

XYZ = np.zeros((5,185))
for i in readFile(filepath):
    if (time >=185):
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
ax.scatter(XYZ[0][:185], XYZ[1][:185], XYZ[2][:185],c='r')
#ax.scatter(XYZ[0][100:185],XYZ[1][100:185],XYZ[2][100:185],c='r')
#ax.scatter(XYZ[0][185:185],XYZ[1][185:185],XYZ[2][185:185],c='g')
plt.show()

X = XYZ[0][0:185]
Y = XYZ[1][0:185]
def residuals(p):
    k,b = p
    return Y - (k*X+b)

r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]

#k =  -0.024755230730224606  
#b =  25.950173117675714
k =  0.0016473714966880708  
b =  -81.6862196326822

print("k = ", k, " b = " ,b)

YY = k* X +b
plt.plot(X, YY, 'r-')


#plt.plot(XYZ[0][0:185], YY, 'r-');
plt.plot(XYZ[0][0:185], XYZ[1][0:185], 'b-');
plt.show();

plt.plot(XYZ[4][0:185],'r-');
plt.show();

Z = XYZ[2][0:185]
def residuals(p):
    k,b = p
    return Z - (k*X+b)

r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]

k =  0.006087397480494191  
b =  1129.1291739801816

ZZ = k* X +b
plt.plot(X, ZZ, 'r-')
plt.plot(X, Z,'b-');
plt.show();
