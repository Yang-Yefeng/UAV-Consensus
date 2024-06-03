import numpy as np
from utils.XML_Operation import *
from utils.collector import data_collector
from controller.BS_FNTSMC import bs_fntsmc_param
import matplotlib.pyplot as plt


def deg2rad(deg: float) -> float:
	"""
    :brief:         omit
    :param deg:     degree
    :return:        radian
    """
	return deg * np.pi / 180.0


def rad2deg(rad: float) -> float:
	"""
    :brief:         omit
    :param rad:     radian
    :return:        degree
    """
	return rad * 180.8 / np.pi


def C(x):
	return np.cos(x)


def S(x):
	return np.sin(x)


def uo_2_ref_angle_throttle(uo: np.ndarray, att: np.ndarray, m: float, g: float):
	phi_d_max = np.pi / 3
	theta_d_max = np.pi / 3
	ux = uo[0]
	uy = uo[1]
	uz = uo[2]
	uf = (uz + g) * m / (C(att[0]) * C(att[1]))

	asin_phi_d = np.clip((ux * np.sin(att[2]) - uy * np.cos(att[2])) * m / uf, -1, 1)
	phi_d = np.arcsin(asin_phi_d)
	phi_d = np.clip(phi_d, -phi_d_max, phi_d_max)

	asin_theta_d = np.clip((ux * np.cos(att[2]) + uy * np.sin(att[2])) * m / (uf * np.cos(phi_d)), -1, 1)
	theta_d = np.arcsin(asin_theta_d)
	theta_d = np.clip(theta_d, -theta_d_max, theta_d_max)

	return phi_d, theta_d, uf


def split_str_2_2d_numpy(s: str, step: int):
	s = s.replace('[', '').replace(']', '').replace(' ', '').split(',', -1)
	row = int(np.round(len(s) / step, 0))
	res = np.zeros((row, step))
	for i in range(row):
		for j in range(step):
			res[i][j] = float(s[i * step + j])
	return res.astype(float)


def split_str_2_1d_numpy(s: str):
	return np.squeeze(split_str_2_2d_numpy(s, 1))


def get_global_variable_from_XML(root: ET.Element) -> dict:
	tag_value = XML_GetTagValue(XML_FindNode('global_variable', root))
	tag_value['dt'] = float(tag_value['dt'])
	tag_value['g_t'] = float(tag_value['g_t'])
	tag_value['g_N'] = int(tag_value['g_N'])
	tag_value['g_tm'] = float(tag_value['g_tm'])
	tag_value['g_ideal'] = False if tag_value['g_ideal'] == '0' else True
	tag_value['uav_num'] = int(tag_value['uav_num'])
	tag_value['g_A'] = split_str_2_2d_numpy(tag_value['g_A'], tag_value['uav_num'])
	tag_value['g_D'] = split_str_2_1d_numpy(tag_value['g_D'])
	tag_value['g_B'] = split_str_2_1d_numpy(tag_value['g_B'])
	return tag_value


def get_uav_param_from_XML(root: ET.Element) -> dict:
	tag_value = XML_GetTagValue(XML_FindNode('uav_param', root))
	tag_value['m'] = float(tag_value['m'])
	tag_value['g'] = float(tag_value['g'])
	tag_value['J'] = split_str_2_1d_numpy(tag_value['J'])
	tag_value['d'] = float(tag_value['d'])
	tag_value['CT'] = float(tag_value['CT'])
	tag_value['CM'] = float(tag_value['CM'])
	tag_value['J0'] = float(tag_value['J0'])
	tag_value['kr'] = float(tag_value['kr'])
	tag_value['kt'] = float(tag_value['kt'])
	tag_value['pos0'] = split_str_2_1d_numpy(tag_value['pos0'])
	tag_value['vel0'] = split_str_2_1d_numpy(tag_value['vel0'])
	tag_value['angle0'] = split_str_2_1d_numpy(tag_value['angle0'])
	tag_value['pqr0'] = split_str_2_1d_numpy(tag_value['pqr0'])
	tag_value['dt'] = float(tag_value['dt'])
	tag_value['time_max'] = float(tag_value['time_max'])
	return tag_value


def get_att_ctrl_parma_from_XML(root: ET.Element) -> bs_fntsmc_param:
	_param = bs_fntsmc_param()
	tag_value = XML_GetTagValue(XML_FindNode('att_ctrl_param', root))
	_param.k1 = split_str_2_1d_numpy(tag_value['k1'])
	_param.k2 = split_str_2_1d_numpy(tag_value['k2'])
	_param.k3 = split_str_2_1d_numpy(tag_value['k3'])
	_param.k4 = split_str_2_1d_numpy(tag_value['k4'])
	_param.k5 = split_str_2_1d_numpy(tag_value['k5'])
	_param.alpha1 = split_str_2_1d_numpy(tag_value['alpha1'])
	_param.alpha2 = split_str_2_1d_numpy(tag_value['alpha2'])
	_param.dim = int(tag_value['dim'])
	_param.dt = float(tag_value['dt'])
	return _param


def get_pos_ctrl_parma_from_XML(root: ET.Element) -> bs_fntsmc_param:
	_param = bs_fntsmc_param()
	tag_value = XML_GetTagValue(XML_FindNode('pos_ctrl_param', root))
	_param.k1 = split_str_2_1d_numpy(tag_value['k1'])
	_param.k2 = split_str_2_1d_numpy(tag_value['k2'])
	_param.k3 = split_str_2_1d_numpy(tag_value['k3'])
	_param.k4 = split_str_2_1d_numpy(tag_value['k4'])
	_param.k5 = split_str_2_1d_numpy(tag_value['k5'])
	_param.alpha1 = split_str_2_1d_numpy(tag_value['alpha1'])
	_param.alpha2 = split_str_2_1d_numpy(tag_value['alpha2'])
	_param.dim = int(tag_value['dim'])
	_param.dt = float(tag_value['dt'])
	return _param


def plot_consensus_pos(data_block: list):
	num = len(data_block)
	plt.figure(figsize=(12, 8))
	for i in range(num):
		plt.subplot(num, 3, 3 * i + 1)
		plt.plot(data_block[i].t, data_block[i].ref_pos[:, 0], 'red')
		plt.plot(data_block[i].t, data_block[i].state[:, 0], 'blue')
		plt.grid(True)
		plt.ylim((-5, 5))
		plt.yticks(np.arange(-5, 5, 1))
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('x')

		plt.subplot(num, 3, 3 * i + 2)
		plt.plot(data_block[i].t, data_block[i].ref_pos[:, 1], 'red')
		plt.plot(data_block[i].t, data_block[i].state[:, 1], 'blue')
		plt.grid(True)
		plt.ylim((-5, 5))
		plt.yticks(np.arange(-5, 5, 1))
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('y')

		plt.subplot(num, 3, 3 * i + 3)
		plt.plot(data_block[i].t, data_block[i].ref_pos[:, 2], 'red')
		plt.plot(data_block[i].t, data_block[i].state[:, 2], 'blue')
		plt.grid(True)
		plt.ylim((-5, 5))
		plt.yticks(np.arange(-5, 5, 1))
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('z')
	plt.subplots_adjust(left=0.05, bottom=0.08, right=0.95, top=0.95, wspace=None, hspace=0.2)


def plot_consensus_vel(data_block: list):
	num = len(data_block)
	plt.figure(figsize=(12, 8))
	for i in range(num):
		plt.subplot(num, 3, 3 * i + 1)
		plt.plot(data_block[i].t, data_block[i].ref_vel[:, 0], 'red')
		plt.plot(data_block[i].t, data_block[i].state[:, 3], 'blue')
		plt.grid(True)
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('vx')

		plt.subplot(num, 3, 3 * i + 2)
		plt.plot(data_block[i].t, data_block[i].ref_vel[:, 1], 'red')
		plt.plot(data_block[i].t, data_block[i].state[:, 4], 'blue')
		plt.grid(True)
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('vy')

		plt.subplot(num, 3, 3 * i + 3)
		plt.plot(data_block[i].t, data_block[i].ref_vel[:, 2], 'red')
		plt.plot(data_block[i].t, data_block[i].state[:, 5], 'blue')
		plt.grid(True)
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('vz')
	plt.subplots_adjust(left=0.05, bottom=0.08, right=0.95, top=0.95, wspace=None, hspace=0.2)


def plot_consensus_att(data_block: list):
	num = len(data_block)
	plt.figure(figsize=(12, 8))
	for i in range(num):
		plt.subplot(num, 3, 3 * i + 1)
		plt.plot(data_block[i].t, data_block[i].ref_angle[:, 0] * 180 / np.pi, 'red')
		plt.plot(data_block[i].t, data_block[i].state[:, 6] * 180 / np.pi, 'blue')
		plt.grid(True)
		plt.ylim((-90, 90))
		plt.yticks(np.arange(-90, 90, 30))
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('roll-phi')

		plt.subplot(num, 3, 3 * i + 2)
		plt.plot(data_block[i].t, data_block[i].ref_angle[:, 1] * 180 / np.pi, 'red')
		plt.plot(data_block[i].t, data_block[i].state[:, 7] * 180 / np.pi, 'blue')
		plt.grid(True)
		plt.ylim((-90, 90))
		plt.yticks(np.arange(-90, 90, 30))
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('pitch-theta')

		plt.subplot(num, 3, 3 * i + 3)
		plt.plot(data_block[i].t, data_block[i].ref_angle[:, 2] * 180 / np.pi, 'red')
		plt.plot(data_block[i].t, data_block[i].state[:, 8] * 180 / np.pi, 'blue')
		plt.grid(True)
		plt.ylim((-100, 100))
		plt.yticks(np.arange(-100, 100, 40))
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('yaw-psi')
	plt.subplots_adjust(left=0.05, bottom=0.08, right=0.95, top=0.95, wspace=None, hspace=0.2)


def plot_consensus_throttle(data_block: list):
	num = len(data_block)
	plt.figure(figsize=(10, 4))
	for i in range(num):
		plt.subplot(1,  num, i + 1)
		plt.plot(data_block[i].t, data_block[i].control[:, 0], 'red')  # 油门
		plt.grid(True)
		plt.title('throttle')
	plt.subplots_adjust(left=0.05, bottom=0.08, right=0.95, top=0.95, wspace=None, hspace=None)


def plot_consensus_torque(data_block: list):
	num = len(data_block)
	plt.figure(figsize=(12, 8))
	for i in range(num):
		plt.subplot(num, 3, 3 * i + 1)
		plt.plot(data_block[i].t, data_block[i].control[:, 1], 'red')  # Tx
		plt.grid(True)
		plt.ylim((-0.3, 0.3))
		plt.yticks(np.arange(-0.3, 0.3, 0.1))
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('Tx')

		plt.subplot(num, 3, 3 * i + 2)
		plt.plot(data_block[i].t, data_block[i].control[:, 2], 'red')  # Ty
		plt.grid(True)
		plt.ylim((-0.3, 0.3))
		plt.yticks(np.arange(-0.3, 0.3, 0.1))
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('Ty')

		plt.subplot(num, 3, 3 * i + 3)
		plt.plot(data_block[i].t, data_block[i].control[:, 3], 'red')  # Tz
		plt.grid(True)
		plt.ylim((-0.3, 0.3))
		plt.yticks(np.arange(-0.3, 0.3, 0.1))
		if i == num - 1:
			plt.xlabel('time(s)')
		if i == 0:
			plt.title('Tz')
	plt.subplots_adjust(left=0.05, bottom=0.08, right=0.95, top=0.95, wspace=None, hspace=0.2)


def plot_consensus_outer_obs(data_block: list):
	num = len(data_block)
	plt.figure(figsize=(12, 8))
	for i in range(num):
		plt.subplot(num, 3, 3 * i + 1)
		plt.plot(data_block[i].t, data_block[i].d_out[:, 0], 'red')
		plt.plot(data_block[i].t, data_block[i].d_out_obs[:, 0], 'blue')
		plt.grid(True)
		if i == num - 1:
			plt.xlabel('time(s)')
		# plt.ylim((-4, 4))
		if i == 0:
			plt.title('observe dx')

		plt.subplot(num, 3, 3 * i + 2)
		plt.plot(data_block[i].t, data_block[i].d_out[:, 1], 'red')
		plt.plot(data_block[i].t, data_block[i].d_out_obs[:, 1], 'blue')
		plt.grid(True)
		if i == num - 1:
			plt.xlabel('time(s)')
		# plt.ylim((-4, 4))
		if i == 0:
			plt.title('observe dy')

		plt.subplot(num, 3, 3 * i + 3)
		plt.plot(data_block[i].t, data_block[i].d_out[:, 2], 'red')
		plt.plot(data_block[i].t, data_block[i].d_out_obs[:, 2], 'blue')
		plt.grid(True)
		if i == num - 1:
			plt.xlabel('time(s)')
		# plt.ylim((-4, 4))
		if i == 0:
			plt.title('observe dz')
	plt.subplots_adjust(left=0.05, bottom=0.08, right=0.95, top=0.95, wspace=None, hspace=0.2)
