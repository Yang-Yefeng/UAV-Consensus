import numpy as np
from utils.utils import *
import sys, os
from typing import Union

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.BS_FNTSMC import bs_fntsmc, bs_fntsmc_param
from controller.BS_FNTSMC_Consensus import bs_fntsmc_consensus
from uav.uav import UAV, uav_param
from utils.utils import *
from utils.collector import data_collector


class usv_consensus:
    def __init__(self,
                 uav_param: uav_param,                  # parameters of a uav
                 ctrl_att_param: bs_fntsmc_param,       # parameters of attitude controller
                 ctrl_pos_param: bs_fntsmc_param,       # parameters of position controller
                 adjacency: Union[np.ndarray, list],    # adjacency of this uav
                 in_degree: float,                      # in-degree of this uav
                 communication: float,                  # communication of this uav
                 obs_att: rd3,                          # observer of attitude loop
                 obs_pos: rd3,                          # observer of position loop
                 is_ideal: bool                         # exist disturbance of not
                 ):
        self.uav = UAV(uav_param)                   # UAV
        self.ctrl_att = bs_fntsmc(ctrl_att_param)   # 内环控制器
        self.ctrl_pos = bs_fntsmc_consensus(ctrl_pos_param)   # 外环控制器
        self.obs_att = obs_att                      # 内环观测器
        self.obs_pos = obs_pos                      # 外环观测器
        self.adjacency = adjacency                  # 这个无人机的邻接矩阵
        self.d = in_degree                          # 这个无人机的入度
        self.b = communication                      # 这个无人机与leader的联通
        self.is_ideal = is_ideal                    # 是否有干扰
        self.data_record = data_collector(N=int(uav_param.time_max / uav_param.dt))
        self.rho_d = 0.         # 该无人机的期望内环指令
        self.dot_rho_d = 0.     # 该无人机的期望内环指令导数
        self.throttle = 0.      # 该无人机的推力

