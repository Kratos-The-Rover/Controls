import numpy as np
import matplotlib.pyplot as plt
import math

#link lengths
l1 = 1
l2 = 1

#joint angles space
q1 = np.linspace(0,np.pi/2,100)
q2 = np.linspace(0,np.pi/2,100)

q = np.array([(x,y) for x in q1 for y in q2])

x_list = []
y_list = []
for q1,q2 in q:
    x = l1 * math.cos(q1) + l2 * math.cos(q1 - q2)
    y = l1 * math.sin(q1) + l2 * math.sin(q1 - q2)
    x_list.append(x)
    y_list.append(y)
    
plt.scatter(x_list,y_list)
plt.grid()
#plot x and y axis
x = np.linspace(0,2,100)
plt.plot(x,np.zeros(x.shape[0]), color="k")
plt.plot(np.zeros(x.shape[0]), x, color="k")
plt.show()
