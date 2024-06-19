import numpy as np
from utils.XML_Operation import *
from utils.utils import *

a = np.array([1,2])
b = np.array([3,4])
c = np.array([5,6])
d = np.array([7,8])
e = np.array([9,0])

# s = np.zeros((5, 2))
# for i, _x in zip(range(5), [a,b,c,d,e]):
# 	s[i][:] = _x
# print(s)

file = './simulation/pos_consensus/global_configuration.xml'

# a = np.array([1,2,3])
# b = np.array([0.5,0.5,0.5])
# print(np.dot(a**2, b))

a = 3
b = 5
c = 2
a += (b * c)
print(a)