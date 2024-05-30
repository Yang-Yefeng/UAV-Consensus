import sys, os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import numpy as np
from controller.BS_FNTSMC import bs_fntsmc, bs_fntsmc_param


class bs_fntsmc_consensus(bs_fntsmc):
    def __init__(self,
                 param: bs_fntsmc_param = None,  # 参数
                 k1: np.ndarray = np.zeros(3),  # backstepping 参数
                 k2: np.ndarray = np.zeros(3),  # 状态误差收敛参数，k2 为 e 的增益
                 k3: np.ndarray = np.zeros(3),  # 状态误差收敛参数，k3 为 sig(e)^\alpha 的增益
                 k4: np.ndarray = np.zeros(3),  # 滑模误差收敛参数，k4 为补偿观测器的增益，理论上是充分小就行
                 k5: np.ndarray = np.zeros(3),  # 滑模误差收敛参数，k5 控制滑模收敛速度
                 alpha1: np.ndarray = 1.5 * np.ones(3),  # 状态误差收敛指数
                 alpha2: np.ndarray = 1.5 * np.ones(3),  # 滑模误差收敛指数
                 dim: int = 3,
                 dt: float = 0.001):
        super(bs_fntsmc_consensus, self).__init__(param, k1, k2, k3, k4, k5, alpha1, alpha2, dim, dt)
        self.control_out_consensus = np.zeros(self.dim)
        self.consensus_e = np.zeros(self.dim)
        self.consensus_de = np.zeros(self.dim)

    def cal_consensus_e_de(self,
                           global_eta: np.ndarray,
                           global_dot_eta: np.ndarray,
                           eta_d: np.ndarray,
                           dot_eta_d: np.ndarray,
                           eta: np.ndarray,
                           dot_eta: np.ndarray,
                           D: float,
                           B: float,
                           A: np.ndarray):
        Lambda = B * eta_d
        for eta_i, a_i in zip(global_eta, A):
            Lambda += a_i * eta_i
        self.consensus_e = (D + B) * eta - Lambda

        dot_Lambda = B * dot_eta_d
        for dot_eta_i, a_i in zip(global_dot_eta, A):
            dot_Lambda += a_i * dot_eta_i
        v_i_d = -(self.k1 * self.consensus_e - dot_Lambda) / (D + B)  # backstepping
        e_v_i = dot_eta - v_i_d
        self.consensus_de = -self.k1 * self.consensus_e + (D + B) * e_v_i

    def control_update_outer_consensus(self,
                                       global_eta: np.ndarray,  # 全局的 eta
                                       global_dot_eta: np.ndarray,  # 全局的 eta 的一阶导
                                       eta_d: np.ndarray,  # 参考 eta
                                       dot_eta_d: np.ndarray,  # 参考 eta 的一阶导
                                       eta: np.ndarray,  # eta
                                       dot_eta: np.ndarray,  # eta 一阶导
                                       D: float,  # 入度
                                       B: float,  # 通信
                                       A: np.ndarray,  # 临接
                                       kt: float,  # uav 风阻系数
                                       m: float,  # uav 质量
                                       obs: float  # 观测器输出
                                       ):
        self.cal_consensus_e_de(global_eta, global_dot_eta, eta_d, dot_eta_d, eta, dot_eta, D, B, A)
        self.s = self.consensus_de + self.k2 * self.consensus_e + self.k3 * self.sig(self.consensus_e, self.alpha1)
        S = (self.k2 + self.k3 * self.alpha1 * self.sig(self.consensus_e, self.alpha1 - 1)) * self.consensus_de - kt * (D + B) * dot_eta / m
        u1 = -S
        u2 = -(D + B) * obs - self.k4 * np.tanh(5 * self.s)
        u3 = -self.k5 * self.sig(self.s, self.alpha2)
        self.control_out_consensus = -(u1 + u2 + u3) / (D + B)
