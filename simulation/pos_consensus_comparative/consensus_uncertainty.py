import numpy as np


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
			
			dp = 0
			dq = 0
			dr = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
		elif 5 < time <= 10:
			Fdx = 1.5
			Fdy = 0.4 * (time - 5.0)
			Fdz = -0.6
			
			dp = 0
			dq = 0
			dr = 0.5 * np.sign(np.round(time - 5) % 2 - 0.5)
		else:
			phi0 = np.pi / 2
			Fdx = 0.5 * np.sin(np.cos(2 * w) * time + phi0) - 1.0 * np.cos(3 * np.sin(w) * time + phi0)
			Fdy = 0.5 * np.sign(np.round(time - 10) % 3 - 1.5) + 0.5 * np.sin(2 * w * time + phi0) - 0.4
			Fdz = 0.5 * np.cos(w * time + phi0) - 1.0 * np.sin(3 * w + time + phi0) + 1.0
			
			dp = 0
			dq = 0
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
			
			dp = 0
			dq = 0
			dr = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
		else:
			phi0 = np.pi / 2
			Fdx = 0.5 * np.sin(np.cos(2 * w) * time + phi0) - 0.3 * np.cos(3 * np.sin(w) * time + phi0)
			Fdy = 0.2 * np.sign(np.round(time - 10) % 3 - 1.5) + 0.5 * np.sin(2 * w * time + phi0) - 0.4 * (time - 10)
			Fdz = 0.5 * np.cos(0.5 * w * time + phi0) - 1.0 * np.sin(3 * w + time + phi0)
			
			dp = 0
			dq = 0
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
			
			dp = 0
			dq = 0
			dr = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
		else:
			phi0 = np.pi / 2
			Fdx = 0.5 * np.sin(np.cos(2 * w) * time + phi0) - 0.4 * np.cos(3 * np.sin(w) * time + phi0)
			Fdy = - 1.0 * np.sqrt(time - 10) / 2
			Fdz = 0.5 * np.cos(0.5 * w * time + phi0) - 0.3 * np.sin(3 * w + time + phi0) + 1.0 * np.sqrt(time - 10)
			
			dp = 0
			dq = 0
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
			
			dp = 0
			dq = 0
			dr = 1.0 * np.cos(2 * w * time + phi0) + 0.4 * np.sin(w * time + phi0)
		elif 5 < time <= 10:
			phi0 = 0.
			Fdx = 1.5
			Fdy = 0.4 * (time - 5.0)
			Fdz = -0.6
			
			dp = 0
			dq = 0
			dr = 0.5
		else:
			phi0 = np.pi / 2
			Fdx = 0.5 * np.sin(np.cos(2 * w) * time + phi0) - 1.0 * np.cos(3 * np.sin(w) * time + phi0)
			Fdy = 0.5 * np.sign(np.round(time - 10) % 2 - 1.0) + 0.8 * np.sin(2 * w * time + phi0) - 0.4
			Fdz = 0.5 * np.cos(w * time + phi0) - 1.0 * np.sin(3 * w + time + phi0) + 1.0
			
			dp = 0
			dq = 0
			dr = 0.5 * np.cos(2 * w * time + phi0) + 0.6 * np.sin(w * time + phi0)
		return np.array([Fdx, Fdy, Fdz, dp, dq, dr])


def consensus_uncertainty(time, is_ideal: bool = False) -> np.ndarray:
	# return generate_uncertainty1(time, is_ideal)
	return np.concatenate((generate_uncertainty1(time, is_ideal),
						   generate_uncertainty2(time, is_ideal),
						   generate_uncertainty3(time, is_ideal),
						   generate_uncertainty4(time, is_ideal)))


def consensus_uncertainty_N(is_ideal: bool = False, dt: float = 0.001, tm: float = 20, num_uav: int = 4) -> np.ndarray:
	res = np.zeros((int(tm / dt), num_uav * 6))
	t = 0.
	i = 0
	while t < tm - dt / 2:
		res[i] = consensus_uncertainty(t, is_ideal)
		i += 1
		t += dt
	return res


def random_uncertainty(dt: float, tm: float, is_ideal: bool = False) -> np.ndarray:
	N = int(np.round(tm / dt))
	d = np.zeros((N, 6)).astype(float)
	if is_ideal:
		return d
	else:
		'''定义干扰的变化基础周期在 T = [2s, 4s]，w = 2pi / T
		5s 一个新信号
		'''
		A_sin = np.random.uniform(0, 1, 6)
		A_cos = np.random.uniform(0, 1, 6)
		w_sin = 2 * np.pi / np.random.uniform(2, 4, 6)
		w_cos = 2 * np.pi / np.random.uniform(2, 4, 6)
		phi0_sin = np.random.uniform(-np.pi / 2, np.pi / 2, 6)
		phi0_cos = np.random.uniform(-np.pi / 2, np.pi / 2, 6)
		for i in range(N):
			d[i][:] = A_sin * np.sin(w_sin * i * dt + phi0_sin) + A_cos * np.sin(w_cos * i * dt + phi0_cos)
		return d


def random_uncertainty_n(n: int, dt: float, tm: float, is_ideal: bool = False) -> np.ndarray:
	N = int(np.round(tm / dt))
	dn = np.zeros((N, 6 * n))
	for i in range(n):
		dn[:, i * 6:(i + 1) * 6] = random_uncertainty(dt, tm, is_ideal)
	return dn


def designed_uncertainty_n(n: int, dt: float, tm: float, is_ideal: bool = False) -> np.ndarray:
	N = int(np.round(tm / dt))
	dn = np.zeros((N, 6 * n))
	T = 5
	w = 2 * np.pi / T
	t1 = np.linspace(0, 10, 1000)	# 前10秒
	t2 = np.linspace(10, 20, 1000)	# 中间10秒
	t3 = np.linspace(20,30,1000)	# 后面10秒
	phi0 = 0.

	# uav1
	Fdx1 = 0.5 * np.sin(w * t1 + phi0) + 0.2 * np.cos(3 * w * t1 + phi0) + 0.2
	Fdy1 = 0.5 * np.cos(w * t1 + phi0) + 0.2 * np.sin(3 * w * t1 + phi0) + 0.4
	Fdz1 = 0.5 * np.sin(w * t1 + phi0) + 0.2 * np.cos(3 * w * t1 + phi0) - 0.5

	Fdx2 = np.sqrt(t2 - 10) + 1.5 * np.cos(np.sin(np.pi * (t2 - 10)))
	Fdy2 = 0.5 * np.sqrt(t2 - 10) + 0.5 * np.cos(np.sin(np.pi * (t2 - 10)))
	Fdz2 = 1.5 * np.sqrt(t2 - 10) - 1.0 * np.cos(np.sin(np.pi * (t2 - 10)))

	Fdx3 = 3.2 * np.ones(1000)
	Fdy3 = 2.0 * np.ones(1000)
	Fdz3 = 0.0 * np.ones(1000)

	Fdx = np.concatenate((Fdx1, Fdx2, Fdx3))
	Fdy = np.concatenate((Fdy1, Fdy2, Fdy3))
	Fdz = np.concatenate((Fdz1, Fdz2, Fdz3))

	dn[:, 0] = Fdx.copy()
	dn[:, 1] = Fdy.copy()
	dn[:, 2] = Fdz.copy()

	# uav2
	Fdx1 = 1.2 * np.ones(1000)
	Fdy1 = 1.0 * np.ones(1000)
	Fdz1 = 0.5 * np.ones(1000)
	Fdx2 = np.sqrt(t2 - 10) + 0.5 * np.cos(np.sin(np.pi * (t2 - 10)))
	Fdy2 = 1.5 * np.sqrt(t2 - 10) + 0.5 * np.cos(np.sin(np.pi * (t2 - 10)))
	Fdz2 = -0.6 * np.sqrt(t2 - 10) + 1.3 * np.cos(np.sin(np.pi * (t2 - 10)))
	Fdx3 = 1.5 * np.sin(w * (t3-20) + phi0) + 1.2 * np.cos(3 * w * (t3-20) + phi0) + 0.
	Fdy3 = 0.7 * np.cos(w * (t3-20) + phi0) + 0.7 * np.sin(3 * w * (t3-20) + phi0) + 0.
	Fdz3 = 0.8 * np.sin(w * (t3-20) + phi0) + 0.1 * np.cos(3 * w * (t3-20) + phi0) - 0.

	Fdx = np.concatenate((Fdx1, Fdx2, Fdx3))
	Fdy = np.concatenate((Fdy1, Fdy2, Fdy3))
	Fdz = np.concatenate((Fdz1, Fdz2, Fdz3))

	dn[:, 6] = Fdx.copy()
	dn[:, 7] = Fdy.copy()
	dn[:, 8] = Fdz.copy()

	# uav3
	Fdx1 = np.sqrt(t1) + 1. * np.cos(np.sin(np.pi * t1))
	Fdy1 = 0.5 * np.sqrt(t1) + 0.7 * np.cos(np.sin(np.pi * t1))
	Fdz1 = 1.5 * np.sqrt(t1) - 1.2 * np.cos(np.sin(np.pi * t1))
	Fdx2 = 0. * np.sin(w * (t2 - 10) + phi0) + 0.2 * np.cos(2 * w * (t2 - 10) + phi0) + 0.25
	Fdy2 = 0.5 * np.cos(w * (t2 - 10) + phi0) + 0. * np.sin(2 * w * (t2 - 10) + phi0) + 0.1
	Fdz2 = 0.85 * np.sin(w * (t2 - 10) + phi0) + 0.2 * np.cos(2 * w * (t2 - 10) + phi0) + 0.5
	Fdx3 = 2.2 * np.ones(1000)
	Fdy3 = 1.0 * np.ones(1000)
	Fdz3 = -1.0 * np.ones(1000)

	Fdx = np.concatenate((Fdx1, Fdx2, Fdx3))
	Fdy = np.concatenate((Fdy1, Fdy2, Fdy3))
	Fdz = np.concatenate((Fdz1, Fdz2, Fdz3))

	dn[:, 12] = Fdx.copy()
	dn[:, 13] = Fdy.copy()
	dn[:, 14] = Fdz.copy()

	# uav4
	Fdx1 = -1.0 * np.sin(3 * w * t1 + phi0) + 0.8 * np.cos(2 * w * t1 + phi0) + 0.2
	Fdy1 = 0.5 * np.cos(3 * w * t1 + phi0) + 0.15 * np.sin(2 * w * t1 + phi0) + 0.
	Fdz1 = 0.85 * np.sin(3 * w * t1 + phi0) + 0.6 * np.cos(2 * w * t1 + phi0) + 0.5
	Fdx2 = 1.2 * np.ones(1000)
	Fdy2 = .0 * np.ones(1000)
	Fdz2 = .0 * np.ones(1000)
	Fdx3 = 0.25 * np.sqrt(t3 - 20) + 1.5 * np.cos(np.sin(np.pi * (t3 - 20)))
	Fdy3 = 0.3 * np.sqrt(t3 - 20) + 0.2 * np.cos(np.sin(np.pi * (t3 - 20)))
	Fdz3 = 1.8 * np.sqrt(t3 - 20) + 0.6 * np.cos(np.sin(np.pi * (t3 - 20)))

	Fdx = np.concatenate((Fdx1, Fdx2, Fdx3))
	Fdy = np.concatenate((Fdy1, Fdy2, Fdy3))
	Fdz = np.concatenate((Fdz1, Fdz2, Fdz3))

	dn[:, 18] = Fdx.copy()
	dn[:, 19] = Fdy.copy()
	dn[:, 20] = Fdz.copy()

	return dn