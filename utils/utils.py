import numpy as np
from utils.XML_Operation import *


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
	# _param = uav_param()
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
