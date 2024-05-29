import numpy as np

a = np.array([1,2])
b = np.array([3,4])
c = np.array([5,6])
d = np.array([7,8])
e = np.array([9,0])

s = np.zeros((5, 2))
for i, _x in zip(range(5), [a,b,c,d,e]):
	s[i][:] = _x
print(s)