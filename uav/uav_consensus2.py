import sys, os
from typing import Union

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.FNTSMC import fntsmc_param, fntsmc
from controller.FNTSMC_Consensus import fntsmc_consensus
from uav.uav import UAV, uav_param
from utils.utils import *
from utils.collector import data_collector


class usv_consensus2:
    def __init__(self,
                 uav_param: uav_param,  # parameters of a uav
                 ctrl_att_param: fntsmc_param,  # parameters of attitude controller
                 ctrl_pos_param: fntsmc_param,  # parameters of position controller
                 adjacency: Union[np.ndarray, list],  # adjacency of this uav
                 in_degree: float,  # in-degree of this uav
                 communication: float,  # communication of this uav
                 obs_att: rd3,  # observer of attitude loop
                 obs_pos: rd3,  # observer of position loop
                 is_ideal: bool  # exist disturbance of not
                 ):
        self.uav = UAV(uav_param)  # UAV
        self.ctrl_att = fntsmc(ctrl_att_param)  # 内环控制器
        self.ctrl_pos = fntsmc_consensus(ctrl_pos_param)  # 外环控制器
        self.obs_att = obs_att  # 内环观测器
        self.obs_pos = obs_pos  # 外环观测器
        self.adjacency = adjacency  # 这个无人机的邻接矩阵
        self.d = in_degree  # 这个无人机的入度
        self.b = communication  # 这个无人机与leader的联通
        self.is_ideal = is_ideal  # 是否有干扰
        self.data_record = data_collector(N=int(uav_param.time_max / uav_param.dt))
        self.rho_d = np.zeros(3)  # 该无人机的期望内环指令
        self.dot_rho_d = np.zeros(3)  # 该无人机的期望内环指令导数
        self.throttle = 0.  # 该无人机的推力
        
        '''计算一致性误差用到，符号定义与 PPT 里面的一致，这里不解释'''
        # self.Lambda = np.zeros(3)
        # self.dot_Lambda = np.zeros(3)
        self.consensus_e = np.zeros(3)
        self.consensus_dot_e = np.zeros(3)
        # self.Lambda_eta = np.zeros(3)
    
    def cal_Lambda(self,
                   g_eta: np.ndarray,  # 所有无人机的位置 2d array
                   g_nu: np.ndarray,  # 所有无人机相对于几何中心的偏移
                   nu: np.ndarray,  # 这个无人机相对于几何中心的偏移
                   eta_d: np.ndarray  # 编队的几何中心
                   ):
        res = (self.d + self.b) * nu + self.b * eta_d
        for _a_ij, _eta_i, _nu_i in zip(self.adjacency, g_eta, g_nu):
            res += _a_ij * (_eta_i - _nu_i)
        return res
    
    def cal_dot_Lambda(self,
                       g_dot_eta: np.ndarray,  # 所有无人机位置导数 2d array
                       g_dot_nu: np.ndarray,  # 所有无人机相对于几何中心的偏移的导数
                       dot_nu: np.ndarray,  # 这个无人机相对于几何中心的偏移的导数
                       dot_eta_d: np.ndarray  # 编队的几何中心的导数
                       ):
        res = (self.d + self.b) * dot_nu + self.b * dot_eta_d
        for _a_ij, _dot_eta_i, _dot_nu_i in zip(self.adjacency, g_dot_eta, g_dot_nu):
            res += _a_ij * (_dot_eta_i - _dot_nu_i)
        return res
    
    def cal_Lambda_eta(self,
                       g_ideal_second_order_dynamics: np.ndarray,  # 所有无人机的二阶动态 (不包含观测器)
                       dotdot_nu: np.ndarray,  # 这个无人机相对于几何中心的偏移的二阶导数
                       dotdot_eta_d: np.ndarray  # 编队的几何中心的二阶导数
                       ):
        res = (self.d + self.b) * dotdot_nu + self.b * dotdot_eta_d
        for _a_ij, _sys in zip(self.adjacency, g_ideal_second_order_dynamics):
            res += _a_ij * _sys
        return res
  
    def cal_consensus_e(self,
                        g_eta: np.ndarray,  # 所有无人机的位置 2d array
                        g_nu: np.ndarray,  # 所有无人机相对于几何中心的偏移
                        nu: np.ndarray,  # 这个无人机相对于几何中心的偏移
                        eta_d: np.ndarray  # 编队的几何中心
                        ):
        self.consensus_e = (self.d + self.b) * self.uav.eta() - self.cal_Lambda(g_eta, g_nu, nu, eta_d)
        
    def cal_consensus_dot_e(self,
                            g_dot_eta: np.ndarray,  # 所有无人机位置导数 2d array
                            g_dot_nu: np.ndarray,  # 所有无人机相对于几何中心的偏移的导数
                            dot_nu: np.ndarray,  # 这个无人机相对于几何中心的偏移的导数
                            dot_eta_d: np.ndarray  # 编队的几何中心的导数
                            ):
        self.consensus_dot_e = (self.d + self.b) * self.uav.dot_eta() - self.cal_dot_Lambda(g_dot_eta, g_dot_nu, dot_nu, dot_eta_d)
