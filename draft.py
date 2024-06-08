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

# root = XML_Load(file)
# global_variable = XML_FindNode('global_variable', root)
# tag_value = XML_GetTagValue(global_variable)
# print(tag_value)
# print(tag_value['dt'], type(tag_value['dt']))
# s = '[[0, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]]'
# J = '[4.212e-3, 4.212e-3, 8.255e-3]'
# res1 = split_str_2_2d_numpy(s,4)
# print(res1)
# res2 = split_str_2_1d_numpy(J)
res2 = np.squeeze(np.array([[2], [1], [2], [3]])).flatten()
print(res2)