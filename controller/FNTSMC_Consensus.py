import sys, os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import numpy as np
from controller.FNTSMC import fntsmc_param, fntsmc


class fntsmc_consensus(fntsmc):
	def __init__(self,
				 param: fntsmc_param = None,  # 参数
				 k1: np.ndarray = np.zeros(3),  # 状态误差收敛参数，k2 为 e 的增益
				 k2: np.ndarray = np.zeros(3),  # 状态误差收敛参数，k3 为 sig(e)^\alpha 的增益
				 k3: np.ndarray = np.zeros(3),  # 滑模误差收敛参数，k4 为补偿观测器的增益，理论上是充分小就行
				 k4: np.ndarray = np.zeros(3),  # 滑模误差收敛参数，k5 控制滑模收敛速度
				 alpha1: np.ndarray = 1.01 * np.ones(3),  # 状态误差收敛指数
				 alpha2: np.ndarray = 1.01 * np.ones(3),  # 滑模误差收敛指数
				 dim: int = 3,
				 dt: float = 0.01):
		super(fntsmc_consensus, self).__init__(param, k1, k2, k3, k4, alpha1, alpha2, dim, dt)
		self.control_out_consensus = np.zeros(self.dim)
	
	def control_update_outer_consensus(self,
									   d: float,
									   b: float,
									   a: np.ndarray,
									   kt: float,
									   m: float,
									   consensus_e: np.ndarray,
									   consensus_de: np.ndarray,
									   Lambda_eta: np.ndarray,
									   dot_eta: np.ndarray,
									   obs: np.ndarray,
									   g_obs: np.ndarray
									   ):
		s = consensus_de + self.k1 * consensus_e + self.k2 * self.sig(consensus_e, self.alpha1)
		sigma = (self.k1 + self.k2 * self.alpha1 * self.sig(consensus_e, self.alpha1 - 1)) * consensus_de
		u1 = -(d + b) * kt / m * dot_eta + sigma - Lambda_eta
		u2 = (d + b) * obs + self.k3 * np.tanh(5 * s) + self.k4 * self.sig(s, self.alpha2)
		for _a_ij, _obs in zip(a, g_obs):
			u2 -= _a_ij * _obs
		self.control_out_consensus = -(u1 + u2) / (d + b)
