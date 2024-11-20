import numpy as np
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import optimize

def readFile(filepath):
    with open(filepath,"r") as f:
        while (True):
            yield f.readline().strip()

filepath = "data_filter.txt"
#list = open(filepath,"r").readlines().strip()
time = 0
list = []

XYZ = np.zeros((5,250))
for i in readFile(filepath):
    if (time >=250):
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

X = XYZ[0][0:250]
Y = XYZ[1][0:250]
def residuals(p):
    k,b = p
    return Y - (k*X+b)

r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]

#k =  -0.024755230730224606  
#b =  25.950173117675714

print("k = ", k, " b = " ,b)

YY = k* X +b


Z = XYZ[2][0:250]
def residuals(p):
    k,b = p
    return Z - (k*X+b)

r = optimize.leastsq(residuals, [1, 0])
k, b = r[0]
# k=-0.003811159340263035  
# b =  1150.6055787187322
ZZ = k* X +b

filepath = "data_origin.txt"
#list = open(filepath,"r").readlines().strip()
time = 0
list = []

XYZ1 = np.zeros((5,250))
for i in readFile(filepath):
    if (time >=250):
        break
    li = [float(j) for j in re.split('\s',i)]
    XYZ1[0][time] = li[0]
    XYZ1[1][time] = li[1]
    XYZ1[2][time] = li[2]
    XYZ1[3][time] = li[3]
    XYZ1[4][time] = li[4]
    time = time + 1
    #print (li)
print (XYZ1)

plt.plot(XYZ[0][0:250], YY, 'r', label='FITTING');
plt.plot(XYZ[0][0:250], XYZ[1][0:250], 'b-', label='FILTER');
plt.plot(XYZ1[0][0:250], XYZ1[1][0:250], 'g-', label='ORIGIN');
plt.legend();
plt.show();

plt.plot(XYZ[4][0:250],'r.-');
plt.show();

plt.plot(X, ZZ, 'r')
plt.plot(X, Z,'b-');
plt.plot(XYZ1[0][0:250], XYZ1[2][0:250], 'g-');
plt.show();
