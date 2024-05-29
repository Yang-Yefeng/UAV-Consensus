# import sys, os
#
# from utils.ref_cmd import *
#
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
import numpy as np

# 这个文件是用来为多智能体仿真生成时用到的干扰。
# 考虑到不同无人机所收到的干扰会不同，所以提前生成并保存好，存在 csv 文件中，以便于在仿真的时候直接加载。
# 仅此而已，没有特殊用途


def generate_uncertainty1(time: float, is_ideal: bool = False) -> np.ndarray:
	if is_ideal:
		return np.array([0, 0, 0, 0, 0, 0]).astype(float)
	else:
		T = 2
		w = 2 * np.pi / T
		phi0 = 0.
		if time <= 5:
			phi0 = 0.
			Fdx = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0) + 0.2
			Fdy = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(3 * w * time + phi0) + 0.4
			Fdz = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0) - 0.5

			dp = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(w * time + phi0)
			dq = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
			dr = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
		elif 5 < time <= 10:
			Fdx = 1.5
			Fdy = 0.4 * (time - 5.0)
			Fdz = -0.6

			dp = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(2 * np.sin(2 * w) * time + phi0)
			dq = 0.5 * np.cos(1.5 * np.sin(2 * w) * time + phi0) + 0.2 * np.sin(w * time + phi0)
			dr = 0.5 * np.sign(np.round(time - 5) % 2 - 0.5)
		else:
			phi0 = np.pi / 2
			Fdx = 0.5 * np.sin(np.cos(2 * w) * time + phi0) - 1.0 * np.cos(3 * np.sin(w) * time + phi0)
			Fdy = 0.5 * np.sign(np.round(time - 10) % 3 - 1.5) + 0.5 * np.sin(2 * w * time + phi0) - 0.4
			Fdz = 0.5 * np.cos(w * time + phi0) - 1.0 * np.sin(3 * w + time + phi0) + 1.0

			dp = 0.5 * np.sin(np.sin(2 * w) * time + phi0) + 0.2 * np.cos(w * time + phi0)
			dq = 1.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0) - 0.7
			dr = 0.5 * np.cos(2 * w * time + phi0) + 0.6 * np.sin(w * time + phi0)
		return np.array([Fdx, Fdy, Fdz, dp, dq, dr])


def generate_uncertainty2(time: float, is_ideal: bool = False) -> np.ndarray:
	if is_ideal:
		return np.array([0, 0, 0, 0, 0, 0]).astype(float)
	else:
		T = 3
		w = 2 * np.pi / T
		if time <= 10:
			phi0 = 0.
			Fdx = 0.5 * np.sin(2 * w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0) + 0.2
			Fdy = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(3 * w * time + phi0) + 0.4
			Fdz = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0) - 0.5

			dp = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(w * time + phi0)
			dq = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
			dr = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
		else:
			phi0 = np.pi / 2
			Fdx = 0.5 * np.sin(np.cos(2 * w) * time + phi0) - 1.0 * np.cos(3 * np.sin(w) * time + phi0)
			Fdy = 1.5 * np.sign(np.round(time - 10) % 3 - 1.5) + 0.5 * np.sin(2 * w * time + phi0) - 0.4 * (time - 10)
			Fdz = 0.5 * np.cos(0.5 * w * time + phi0) - 1.0 * np.sin(3 * w + time + phi0) + 1.0 * (time - 10) ** 2

			dp = 0.5 * np.sin(np.sin(2 * w) * time + phi0) + 0.2 * np.cos(w * time + phi0)
			dq = 1.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0) - 0.7
			dr = 0.5 * np.cos(2 * w * time + phi0) + 0.6 * np.sin(w * time + phi0)
		return np.array([Fdx, Fdy, Fdz, dp, dq, dr])


def generate_uncertainty3(time: float, is_ideal: bool = False) -> np.ndarray:
	if is_ideal:
		return np.array([0, 0, 0, 0, 0, 0]).astype(float)
	else:
		T = 5
		w = 2 * np.pi / T
		if time <= 10:
			phi0 = 0.
			Fdx = 0.5 * np.sin(2 * w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0) + 0.2
			Fdy = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(3 * w * time + phi0) + 0.4
			Fdz = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0) - 0.5

			dp = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(w * time + phi0)
			dq = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
			dr = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
		else:
			phi0 = np.pi / 2
			Fdx = 0.5 * np.sin(np.cos(2 * w) * time + phi0) - 1.0 * np.cos(3 * np.sin(w) * time + phi0)
			Fdy = - 1.0 * np.sqrt(time - 10)
			Fdz = 0.5 * np.cos(0.5 * w * time + phi0) - 1.0 * np.sin(3 * w + time + phi0) + 1.0 * (time - 10) ** 2

			dp = 0.5 * np.sin(np.sin(2 * w) * time + phi0) + 0.2 * np.cos(w * time + phi0)
			dq = 1.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0) - 0.7
			dr = 0.5 * np.cos(2 * w * time + phi0) + 1.6 * np.sin(w * time + phi0)
		return np.array([Fdx, Fdy, Fdz, dp, dq, dr])


def generate_uncertainty4(time: float, is_ideal: bool = False) -> np.ndarray:
	if is_ideal:
		return np.array([0, 0, 0, 0, 0, 0]).astype(float)
	else:
		T = 2
		w = 2 * np.pi / T
		if time <= 5:
			phi0 = 0.
			Fdx = 0.5 * np.sin(1.5 * w * time + phi0) + 0.2 * np.cos(2 * w * time + phi0) + 1.2
			Fdy = 1.5 * np.cos(1.5 * w * time + phi0) + 0.4 * np.sin(2 * w * time + phi0) + 0.4
			Fdz = 0.8 * np.sin(1.5 * w * time + phi0) + 0.7 * np.cos(2 * w * time + phi0) - 0.5

			dp = 0.5 * np.sin(2 * w * time + phi0) + 0.4 * np.cos(w * time + phi0)
			dq = 0.5 * np.cos(2 * w * time + phi0) + 0.6 * np.sin(w * time + phi0)
			dr = 1.0 * np.cos(2 * w * time + phi0) + 0.4 * np.sin(w * time + phi0)
		elif 5 < time <= 10:
			phi0 = 0.
			Fdx = 1.5
			Fdy = 0.4 * (time - 5.0)
			Fdz = -0.6 * (time - 5) ** 2

			dp = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(2 * np.sin(2 * w) * time + phi0)
			dq = 0.5 * np.cos(1.5 * np.sin(2 * w) * time + phi0) + 0.2 * np.sin(w * time + phi0)
			dr = 0.5
		else:
			phi0 = np.pi / 2
			Fdx = 0.5 * np.sin(np.cos(2 * w) * time + phi0) - 1.0 * np.cos(3 * np.sin(w) * time + phi0)
			Fdy = 0.5 * np.sign(np.round(time - 10) % 2 - 1.0) + 0.8 * np.sin(2 * w * time + phi0) - 0.4
			Fdz = 0.5 * np.cos(w * time + phi0) - 1.0 * np.sin(3 * w + time + phi0) + 1.0

			dp = 0.5 * np.sin(np.sin(2 * w) * time + phi0) + 0.2 * np.cos(w * time + phi0)
			dq = 1.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0) - 0.7
			dr = 0.5 * np.cos(2 * w * time + phi0) + 0.6 * np.sin(w * time + phi0)
		return np.array([Fdx, Fdy, Fdz, dp, dq, dr])


def consensus_uncertainty(time, is_ideal: bool = False) -> np.ndarray:
	return np.concatenate((generate_uncertainty1(time, is_ideal),
						   generate_uncertainty2(time, is_ideal),
						   generate_uncertainty3(time, is_ideal),
						   generate_uncertainty4(time, is_ideal)))


def consensus_uncertainty_N(is_ideal: bool = False, dt: float = 0., tm: float = 20, num_uav: int = 4) -> np.ndarray:
	res = np.zeros((int(tm / dt), num_uav * 6))
	t = 0.
	i = 0
	while t < tm - dt / 2:
		res[i] = consensus_uncertainty(t, is_ideal)
		t += dt
	return res
