import numpy as np

from uav.uav import UAV, uav_param


class multi_uav:
    def __init__(self,  uav_params: list, A: np.ndarray, D:np.ndarray, B: np.ndarray, num: int=4):
        """
        :param uav_params:      所有无人机的参数
        :param A:               邻接矩阵
        :param D:               入度矩阵
        :param B:               leader 连接矩阵
        :param num:             无人机数量
        """
        self.uav_num = num
        self.uavs = []
        for i in range(self.uav_num):
            self.uavs.append(UAV(uav_params[i]))
        self.A = A
        self.D = D
        self.L = self.D - self.A
        self.B = B
