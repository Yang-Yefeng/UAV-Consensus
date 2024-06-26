import numpy as np
import os
import pandas as pd

from utils.utils import *
from utils.PPOActor import Normalization


class uav_param:
	def __init__(self,
				 m: float = 0.8,  # 无人机质量
				 g: float = 9.8,  # 重力加速度
				 J: np.ndarray = np.array([4.212e-3, 4.212e-3, 8.255e-3]),  # 转动惯量
				 d: float = 0.12,  # 机臂长度 'X'构型
				 CT: float = 2.168e-6,  # 螺旋桨升力系数
				 CM: float = 2.136e-8,  # 螺旋桨力矩系数
				 J0: float = 1.01e-5,  # 电机和螺旋桨的转动惯量
				 kr: float = 1e-3,  # 旋转阻尼系数
				 kt: float = 1e-3,  # 平移阻尼系数
				 pos0: np.ndarray = np.array([0, 0, 0]),
				 vel0: np.ndarray = np.array([0, 0, 0]),
				 angle0: np.ndarray = np.array([0, 0, 0]),
				 pqr0: np.ndarray = np.array([0, 0, 0]),
				 dt=0.001,
				 time_max=30,
				 from_dict: dict = None):
		if from_dict:
			self.m = from_dict['m']
			self.g = from_dict['g']
			self.J = from_dict['J']
			self.d = from_dict['d']
			self.CT = from_dict['CT']
			self.CM = from_dict['CM']
			self.J0 = from_dict['J0']
			self.kr = from_dict['kr']
			self.kt = from_dict['kt']
			self.pos0 = from_dict['pos0']
			self.vel0 = from_dict['vel0']
			self.angle0 = from_dict['angle0']
			self.pqr0 = from_dict['pqr0']
			self.dt = from_dict['dt']
			self.time_max = from_dict['time_max']
		else:
			self.m = m
			self.g = g
			self.J = J
			self.d = d
			self.CT = CT
			self.CM = CM
			self.J0 = J0
			self.kr = kr
			self.kt = kt
			self.pos0 = pos0
			self.vel0 = vel0
			self.angle0 = angle0
			self.pqr0 = pqr0
			self.dt = dt
			self.time_max = time_max
	
	def print_param(self):
		print('    m   : ', self.m)
		print('    g   : ', self.g)
		print('    J   : ', self.J)
		print('    d   : ', self.d)
		print('   CT   : ', self.CT)
		print('   CM   : ', self.CM)
		print('   J0   : ', self.J0)
		print('   kr   : ', self.kr)
		print('   kt   : ', self.kt)
		print('  pos0  : ', self.pos0)
		print('  vel0  : ', self.vel0)
		print(' angle0 : ', self.angle0)
		print('  pqr0  : ', self.pqr0)
		print('   dt   : ', self.dt)
		print('time_max: ', self.time_max)


class UAV:
	def __init__(self, param: uav_param):
		self.m = param.m  # 无人机质量
		self.g = param.g  # 重力加速度
		self.J = param.J  # 转动惯量
		self.d = param.d  # 机臂长度 'X'构型
		self.CT = param.CT  # 螺旋桨升力系数
		self.CM = param.CM  # 螺旋桨力矩系数
		self.J0 = param.J0  # 电机和螺旋桨的转动惯量
		self.kr = param.kr  # 旋转阻尼系数
		self.kt = param.kt  # 平移阻尼系数
		self.rotor_wMax = 25000.0 * 2 * np.pi / 60  # 电机最大转速，25000rpm
		
		[self.x, self.y, self.z] = param.pos0[:]
		[self.vx, self.vy, self.vz] = param.vel0[:]
		[self.phi, self.theta, self.psi] = param.angle0[:]
		[self.p, self.q, self.r] = param.pqr0[:]
		
		self.dt = param.dt
		self.n = 0
		self.time = 0.
		self.time_max = param.time_max
		'''physical parameters'''
		
		'''control'''
		self.throttle = self.m * self.g  # 油门
		self.torque = np.array([0., 0., 0.]).astype(float)  # 转矩
		
		self.control = np.array([self.m * self.g, 0., 0., 0.]).astype(float)
		'''control'''
		
		'state limitation'
		self.pos_min = np.array([-10, -10, -10])
		self.pos_max = np.array([10, 10, 10])
		self.vel_min = np.array([-10, -10, -10])
		self.vel_max = np.array([10, 10, 10])
		self.angle_min = np.array([-deg2rad(80), -deg2rad(80), -deg2rad(180)])
		self.angle_max = np.array([deg2rad(80), deg2rad(80), deg2rad(180)])
		self.dangle_min = np.array([-deg2rad(360 * 3), -deg2rad(360 * 3), -deg2rad(360 * 2)])
		self.dangle_max = np.array([deg2rad(360 * 3), deg2rad(360 * 3), deg2rad(360 * 2)])
		'state limitation'
		
		self.reward = 0.
		self.sum_reward = 0.
		self.Q_att = np.array([1., 1., 1.])  # 角度误差惩罚
		self.Q_pqr = np.array([0.01, 0.01, 0.01])  # 角速度误差惩罚
		self.R_att = np.array([0.0, 0.0, 0.0])  # 期望加速度输出 (即控制输出) 惩罚
		
		self.Q_pos = np.array([1., 1., 1.])
		self.Q_vel = np.array([0.01, 0.01, 0.01])
		self.R_pos = np.array([0.0, 0.0, 0.0])
		
		self.project_path = os.path.dirname(os.path.abspath(__file__)) + '/../'
		self.pos_state_norm = Normalization(6)
		self.att_state_norm = Normalization(6)
	
	def ode(self, xx: np.ndarray, dis: np.ndarray):
		"""
        :param dis:             干扰
        :param xx:              状态
        :return:                状态的导数
        """
		'''
        在微分方程里面的状态 X = [x y z vx vy vz phi theta psi p q r] 一共12个
        定义惯性系到机体系的旋转矩阵
        R_i_b1 = np.array([[math.cos(self.psi), math.sin(self.psi), 0],
                           [-math.sin(self.psi), math.cos(self.psi), 0],
                           [0, 0, 1]])  # 从惯性系到b1系，旋转偏航角psi
        R_b1_b2 = np.array([[math.cos(self.theta), 0, -math.sin(self.theta)],
                            [0, 1, 0],
                            [math.sin(self.theta), 0, math.cos(self.theta)]])  # 从b1系到b2系，旋转俯仰角theta
        R_b2_b = np.array([[1, 0, 0],
                           [0, math.cos(self.phi), math.sin(self.phi)],
                           [0, -math.sin(self.phi), math.cos(self.phi)]])  # 从b2系到b系，旋转滚转角phi
        R_i_b = np.matmul(R_b2_b, np.matmul(R_b1_b2, R_i_b1))  # 从惯性系到机体系的转换矩阵
        R_b_i = R_i_b.T  # 从机体系到惯性系的转换矩阵
        e3_i = np.array([0, 0, 1])  # 惯性系下的Z轴基向量
        # dx = v
        # dv = g*e3_i + f/m*
        '''
		[_x, _y, _z, _vx, _vy, _vz, _phi, _theta, _psi, _p, _q, _r] = xx[0:12]
		
		'''至此，已经计算出理想控制量的，油门 + 三转矩'''
		self.throttle = self.control[0]
		self.torque = self.control[1: 4]
		
		'''至此，已经计算出实际控制量的，油门 + 三转矩'''
		
		'''1. 无人机绕机体系旋转的角速度p q r 的微分方程'''
		self.J0 = 0.  # 不考虑陀螺力矩，用于分析观测器的效果
		dp = (-self.kr * _p - _q * _r * (self.J[2] - self.J[1]) + self.torque[0]) / self.J[0] + dis[3]
		dq = (-self.kr * _q - _p * _r * (self.J[0] - self.J[2]) + self.torque[1]) / self.J[1] + dis[4]
		dr = (-self.kr * _r - _p * _q * (self.J[1] - self.J[0]) + self.torque[2]) / self.J[2] + dis[5]
		# if dr == np.nan:
		#     print(self.time)
		'''1. 无人机绕机体系旋转的角速度p q r 的微分方程'''
		
		'''2. 无人机在惯性系下的姿态角 phi theta psi 的微分方程'''
		_R_pqr2diner = np.array([[1, np.tan(_theta) * np.sin(_phi), np.tan(_theta) * np.cos(_phi)],
								 [0, np.cos(_phi), -np.sin(_phi)],
								 [0, np.sin(_phi) / np.cos(_theta), np.cos(_phi) / np.cos(_theta)]])
		[dphi, dtheta, dpsi] = np.dot(_R_pqr2diner, [_p, _q, _r]).tolist()
		'''2. 无人机在惯性系下的姿态角 phi theta psi 的微分方程'''
		
		'''3. 无人机在惯性系下的位置 x y z 和速度 vx vy vz 的微分方程'''
		[dx, dy, dz] = [_vx, _vy, _vz]
		dvx = (self.throttle * (np.cos(_psi) * np.sin(_theta) * np.cos(_phi) + np.sin(_psi) * np.sin(_phi))
			   - self.kt * _vx + dis[0]) / self.m
		dvy = (self.throttle * (np.sin(_psi) * np.sin(_theta) * np.cos(_phi) - np.cos(_psi) * np.sin(_phi))
			   - self.kt * _vy + dis[1]) / self.m
		dvz = -self.g + (self.throttle * np.cos(_phi) * np.cos(_theta)
						 - self.kt * _vz + dis[2]) / self.m
		'''3. 无人机在惯性系下的位置 x y z 和速度 vx vy vz 的微分方程'''
		
		return np.array([dx, dy, dz, dvx, dvy, dvz, dphi, dtheta, dpsi, dp, dq, dr])
	
	def rk44(self, action: np.ndarray, dis: np.ndarray, n: int = 10, att_only: bool = False):
		self.control = action
		h = self.dt / n  # RK-44 解算步长
		# tt = self.time + self.dt
		
		cc = 0
		xx = self.uav_state_call_back()
		while cc < n:
			K1 = h * self.ode(xx, dis)
			K2 = h * self.ode(xx + K1 / 2, dis)
			K3 = h * self.ode(xx + K2 / 2, dis)
			K4 = h * self.ode(xx + K3, dis)
			xx = xx + (K1 + 2 * K2 + 2 * K3 + K4) / 6
			cc += 1
		if att_only:
			xx[0:6] = np.zeros(6)[:]
		self.set_state(xx)
		self.time += self.dt
		if self.psi > np.pi:  # 如果角度超过 180 度
			self.psi -= 2 * np.pi
		if self.psi < -np.pi:  # 如果角度小于 -180 度
			self.psi += 2 * np.pi
		self.n += 1  # 拍数 +1
	
	def uav_state_call_back(self):
		return np.array([self.x, self.y, self.z, self.vx, self.vy, self.vz, self.phi, self.theta, self.psi, self.p, self.q, self.r])
	
	def uav_pos_vel_call_back(self):
		return np.array([self.x, self.y, self.z, self.vx, self.vy, self.vz])
	
	def uav_att_pqr_call_back(self):
		return np.array([self.phi, self.theta, self.psi, self.p, self.q, self.r])
	
	def uav_pos(self):
		return np.array([self.x, self.y, self.z])
	
	def uav_vel(self):
		return np.array([self.vx, self.vy, self.vz])
	
	def uav_att(self):
		return np.array([self.phi, self.theta, self.psi])
	
	def uav_pqr(self):
		return np.array([self.p, self.q, self.r])
	
	def uav_dot_att(self):
		return np.dot(self.W(), self.uav_pqr())
	
	def set_state(self, xx: np.ndarray):
		[self.x, self.y, self.z, self.vx, self.vy, self.vz, self.phi, self.theta, self.psi, self.p, self.q, self.r] = xx[:]
	
	def W(self):
		return np.array([[1, np.sin(self.phi) * np.tan(self.theta), np.cos(self.phi) * np.tan(self.theta)],
						 [0, np.cos(self.phi), -np.sin(self.phi)],
						 [0, np.sin(self.phi) / np.cos(self.theta), np.cos(self.phi) / np.cos(self.theta)]])
	
	def f2(self):
		"""
        :brief:  [(kr * p + qr * (Iyy - Izz)) / Ixx]
                 [(kr * q + pr * (Izz - Ixx)) / Iyy]
                 [(kr * r + pq * (Ixx - Iyy)) / Izz]
        :return: f2(rho_2)
        """
		_f2_rho2 = np.array([0, 0, 0]).astype(float)
		_f2_rho2[0] = (self.kr * self.p + self.q * self.r * (self.J[1] - self.J[2])) / self.J[0]
		_f2_rho2[1] = (self.kr * self.q + self.p * self.r * (self.J[2] - self.J[0])) / self.J[1]
		_f2_rho2[2] = (self.kr * self.r + self.p * self.q * (self.J[0] - self.J[1])) / self.J[2]
		return _f2_rho2
	
	def J_inv(self):
		"""
        :brief:  [1/Jxx    0       0 ]
                 [  0    1/Jyy     0 ]
                 [  0      0    1/Jzz]
        :return: h(rho_1)
        """
		_g = np.zeros((3, 3)).astype(float)
		_g[0][0] = 1 / self.J[0]
		_g[1][1] = 1 / self.J[1]
		_g[2][2] = 1 / self.J[2]
		return _g
	
	def f_rho(self):
		"""
        :return:
        """
		f_rho = np.array([0, 0, 0]).astype(float)
		f_rho[0] = -self.kr * self.p - self.q * self.r * (self.J[2] - self.J[1])
		f_rho[1] = -self.kr * self.q - self.p * self.r * (self.J[0] - self.J[2])
		f_rho[2] = -self.kr * self.r - self.p * self.q * (self.J[1] - self.J[0])
		return f_rho
	
	def rho1(self):
		return np.array([self.phi, self.theta, self.psi])
	
	def rho2(self):
		return np.array([self.p, self.q, self.r])
	
	def dot_rho1(self):
		return np.dot(self.W(), self.rho2())
	
	def dot_rho2(self, uncertainty: np.ndarray):
		# Fdx, Fdy, Fdz, dp, dq, dr
		dp, dq, dr = uncertainty[3], uncertainty[4], uncertainty[5]
		return self.f2() + np.dot(self.J_inv(), self.torque) + np.array([dp, dq, dr])
	
	def dW(self):
		dot_rho1 = self.dot_rho1()  # dphi dtheta dpsi
		_dot_f1_rho1 = np.zeros((3, 3)).astype(float)
		_dot_f1_rho1[0][1] = dot_rho1[0] * np.tan(self.theta) * np.cos(self.phi) + dot_rho1[1] * np.sin(self.phi) / np.cos(self.theta) ** 2
		_dot_f1_rho1[0][2] = -dot_rho1[0] * np.tan(self.theta) * np.sin(self.phi) + dot_rho1[1] * np.cos(self.phi) / np.cos(self.theta) ** 2
		
		_dot_f1_rho1[1][1] = -dot_rho1[0] * np.sin(self.phi)
		_dot_f1_rho1[1][2] = -dot_rho1[0] * np.cos(self.phi)
		
		_temp1 = dot_rho1[0] * np.cos(self.phi) * np.cos(self.theta) + dot_rho1[1] * np.sin(self.phi) * np.sin(self.theta)
		_dot_f1_rho1[2][1] = _temp1 / np.cos(self.theta) ** 2
		
		_temp2 = -dot_rho1[0] * np.sin(self.phi) * np.cos(self.theta) + dot_rho1[1] * np.cos(self.phi) * np.sin(self.theta)
		_dot_f1_rho1[2][2] = _temp2 / np.cos(self.theta) ** 2
		return _dot_f1_rho1
	
	def eta(self):
		return np.array([self.x, self.y, self.z])
	
	def dot_eta(self):
		return np.array([self.vx, self.vy, self.vz])
	
	def A(self):
		return self.control[0] / self.m * np.array([C(self.phi) * C(self.psi) * S(self.theta) + S(self.phi) * S(self.psi),
													C(self.phi) * S(self.psi) * S(self.theta) - S(self.phi) * C(self.psi),
													C(self.phi) * C(self.theta)]) - np.array([0., 0., self.g])
	
	def second_order_att_dynamics(self) -> np.ndarray:
		return np.dot(self.dW(), self.rho2()) + np.dot(self.W(), self.f2())
	
	def omega(self):
		return self.rho2()
	
	def A_rho(self):
		return np.dot(self.dW(), self.rho2()) + np.dot(self.W(), np.dot(self.J_inv(), self.f_rho()))
	
	def B_rho(self):
		return np.dot(self.W(), self.J_inv())
	
	def A_omega(self):
		return np.dot(self.J_inv(), self.f_rho())
	
	def B_omega(self):
		return self.J_inv()
	
	def load_pos_normalizer(self, file):
		data = pd.read_csv(file, header=0).to_numpy()
		self.pos_state_norm.running_ms.n = data[0, 0]
		self.pos_state_norm.running_ms.mean = data[:, 1]
		self.pos_state_norm.running_ms.std = data[:, 2]
		self.pos_state_norm.running_ms.S = data[:, 3]
		self.pos_state_norm.running_ms.n = data[0, 4]
		self.pos_state_norm.running_ms.mean = data[:, 5]
		self.pos_state_norm.running_ms.std = data[:, 6]
		self.pos_state_norm.running_ms.S = data[:, 7]
	
	def load_att_normalizer(self, file):
		data = pd.read_csv(file, header=0).to_numpy()
		self.att_state_norm.running_ms.n = data[0, 0]
		self.att_state_norm.running_ms.mean = data[:, 1]
		self.att_state_norm.running_ms.std = data[:, 2]
		self.att_state_norm.running_ms.S = data[:, 3]
		self.att_state_norm.running_ms.n = data[0, 4]
		self.att_state_norm.running_ms.mean = data[:, 5]
		self.att_state_norm.running_ms.std = data[:, 6]
		self.att_state_norm.running_ms.S = data[:, 7]
	
	def get_reward_att(self, r, dr, acc):
		_e_att = self.uav_att() - r
		_e_pqr = self.uav_dot_att() - dr
		
		u_att = -np.dot(_e_att ** 2, self.Q_att)
		u_pqr = -np.dot(_e_pqr ** 2, self.Q_pqr)
		u_acc = -np.dot(acc ** 2, self.R_att)
		
		return u_att + u_pqr + u_acc
	
	def get_reward_pos(self, r, dr, acc):
		_e_pos = self.uav_att() - r
		_e_vel = self.uav_dot_att() - dr
		
		u_att = -np.dot(_e_pos ** 2, self.Q_pos)
		u_pqr = -np.dot(_e_vel ** 2, self.Q_vel)
		u_acc = -np.dot(acc ** 2, self.R_pos)
		
		return u_att + u_pqr + u_acc
		
	def reset_with_param(self, param: uav_param = None):
		self.reward = 0.
		self.sum_reward = 0.
		self.n = 0
		self.time = 0.
		self.throttle = self.m * self.g  # 油门
		self.torque = np.array([0., 0., 0.]).astype(float)  # 转矩
		self.control = np.array([self.m * self.g, 0., 0., 0.]).astype(float)
		if param is not None:
			[self.x, self.y, self.z] = param.pos0[:]
			[self.vx, self.vy, self.vz] = param.vel0[:]
			[self.phi, self.theta, self.psi] = param.angle0[:]
			[self.p, self.q, self.r] = param.pqr0[:]
		else:
			self.set_state(np.zeros(12))
