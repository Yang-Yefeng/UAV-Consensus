import numpy as np


def ref_inner(time, amplitude: np.ndarray, period: np.ndarray, bias_a: np.ndarray, bias_phase: np.ndarray):
    """
    :param time:        time
    :param amplitude:   amplitude
    :param period:      period
    :param bias_a:      amplitude bias
    :param bias_phase:  phase bias
    :return:            reference attitude angles and their 1st - 3rd derivatives
                        [phi_d, theta_d, psi_d]
                        [dot_phi_d, dot_theta_d, dot_psi_d]
                        [dot2_phi_d, dot2_theta_d, dot2_psi_d]
                        [dot3_phi_d, dot3_theta_d, dot3_psi_d]
    """
    w = 2 * np.pi / period
    _r = amplitude * np.sin(w * time + bias_phase) + bias_a
    _dr = amplitude * w * np.cos(w * time + bias_phase)
    _ddr = -amplitude * w ** 2 * np.sin(w * time + bias_phase)
    return _r, _dr, _ddr


def ref_inner_all(a: np.ndarray, p: np.ndarray, ba: np.ndarray, bp: np.ndarray, t_max: float, dt: float):
    t = np.linspace(0, t_max, int(t_max / dt) + 1)
    w = 2 * np.pi / p
    r_phi = ba[0] + a[0] * np.sin(w[0] * t + bp[0])
    r_theta = ba[1] + a[1] * np.sin(w[1] * t + bp[1])
    r_psi = ba[2] + a[2] * np.sin(w[2] * t + bp[2])

    r_d_phi = a[0] * w[0] * np.cos(w[0] * t + bp[0])
    r_d_theta = a[1] * w[1] * np.cos(w[1] * t + bp[1])
    r_d_psi = a[2] * w[2] * np.cos(w[2] * t + bp[2])

    r_dd_phi = -a[0] * w[0] ** 2 * np.sin(w[0] * t + bp[0])
    r_dd_theta = -a[1] * w[1] ** 2 * np.sin(w[1] * t + bp[1])
    r_dd_psi = -a[2] * w[2] ** 2 * np.sin(w[2] * t + bp[2])
    return np.vstack((r_phi, r_theta, r_psi)).T, np.vstack((r_d_phi, r_d_theta, r_d_psi)).T, np.vstack((r_dd_phi, r_dd_theta, r_dd_psi)).T


def ref_uav(time: float, amplitude: np.ndarray, period: np.ndarray, bias_a: np.ndarray, bias_phase: np.ndarray):
    """
    :param time:        time
    :param amplitude:   amplitude
    :param period:      period
    :param bias_a:      amplitude bias
    :param bias_phase:  phase bias
    :return:            reference position and yaw angle and their 1st - 3rd derivatives
                        [x_d, y_d, z_d, yaw_d]
                        [dot_x_d, dot_y_d, dot_z_d, dot_yaw_d]
                        [dot2_x_d, dot2_y_d, dot2_z_d, dot2_yaw_d]
                        [dot3_x_d, dot3_y_d, dot3_z_d, dot3_yaw_d]
    """
    w = 2 * np.pi / period
    _r = amplitude * np.sin(w * time + bias_phase) + bias_a
    _dr = amplitude * w * np.cos(w * time + bias_phase)
    _ddr = -amplitude * w ** 2 * np.sin(w * time + bias_phase)
    return _r, _dr, _ddr


def offset_uav_n(time: float, amplitude: np.ndarray, period: np.ndarray, bias_a: np.ndarray, bias_phase: np.ndarray):
    '''所有参数，除了时间之外，都是二维的 numpy，每一行表示一个无人机，行里面的每一列分别表示 x y z'''
    w = 2 * np.pi / period
    _off = amplitude * np.sin(w * time + bias_phase) + bias_a
    _doff = amplitude * w * np.cos(w * time + bias_phase)
    _ddoff = -amplitude * w ** 2 * np.sin(w * time + bias_phase)
    return _off, _doff, _ddoff


# def generate_uncertainty(time: float, is_ideal: bool = False) -> np.ndarray:
# 	"""
#     :param time:        time
#     :param is_ideal:    ideal or not
#     :return:            Fdx, Fdy, Fdz, dp, dq, dr
#     """
# 	if is_ideal:
# 		return np.array([0, 0, 0, 0, 0, 0]).astype(float)
# 	else:
# 		T = 2
# 		w = 2 * np.pi / T
# 		phi0 = 0.
# 		if time <= 5:
# 			phi0 = 0.
# 			Fdx = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0) + 0.2
# 			Fdy = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(3 * w * time + phi0) + 0.4
# 			Fdz = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0) - 0.5
#
# 			dp = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(w * time + phi0)
# 			dq = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
# 			dr = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)
# 		elif 5 < time <= 10:
# 			Fdx = 1.5
# 			Fdy = 0.4 * (time - 5.0)
# 			Fdz = -0.6
#
# 			dp = 0.5 * np.sin(w * time + phi0) + 0.2 * np.cos(2 * np.sin(2 * w) * time + phi0)
# 			dq = 0.5 * np.cos(1.5 * np.sin(2 * w) * time + phi0) + 0.2 * np.sin(w * time + phi0)
# 			dr = 0.5 * np.sign(np.round(time - 5) % 2 - 0.5)
# 		else:
# 			phi0 = np.pi / 2
# 			Fdx = 0.5 * np.sin(np.cos(2 * w) * time + phi0) - 1.0 * np.cos(3 * np.sin(w) * time + phi0)
# 			Fdy = 0.5 * np.sign(np.round(time - 10) % 3 - 1.5) + 0.5 * np.sin(2 * w * time + phi0) - 0.4
# 			Fdz = 0.5 * np.cos(w * time + phi0) - 1.0 * np.sin(3 * w + time + phi0) + 1.0
#
# 			dp = 0.5 * np.sin(np.sin(2 * w) * time + phi0) + 0.2 * np.cos(w * time + phi0)
# 			dq = 1.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0) - 0.7
# 			dr = 0.5 * np.cos(2 * w * time + phi0) + 0.6 * np.sin(w * time + phi0)
#
# 		return np.array([Fdx, Fdy, Fdz, dp, dq, dr])


def generate_uncertainty(time: float, is_ideal: bool = False) -> np.ndarray:
    """
    :param time:        time
    :param is_ideal:    ideal or not
    :return:            Fdx, Fdy, Fdz, dp, dq, dr
    """
    if is_ideal:
        return np.array([0, 0, 0, 0, 0, 0]).astype(float)
    else:
        T = 5
        w = 2 * np.pi / T
        phi0 = 0.
        Fdx = 1.5 * np.sin(w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0)
        Fdy = 1.5 * np.cos(w * time + phi0) + 0.2 * np.sin(3 * w * time + phi0)
        Fdz = 1.5 * np.sin(w * time + phi0) + 0.2 * np.cos(3 * w * time + phi0)

        dp = 0.
        dq = 0.
        dr = 0.5 * np.cos(w * time + phi0) + 0.2 * np.sin(w * time + phi0)

        return np.array([Fdx, Fdy, Fdz, dp, dq, dr])


def ref_uav_sequence(dt: float,
                     tm: float,
                     amplitude: np.ndarray,
                     period: np.ndarray,
                     bias_a: np.ndarray,
                     bias_phase: np.ndarray):
    w = 2 * np.pi / period
    N = int(np.round(tm / dt))
    _r = np.zeros((N, 4))
    _dr = np.zeros((N, 4))
    _ddr = np.zeros((N, 4))
    for i in range(N):
        _r[i, :] = amplitude * np.sin(w * i * dt + bias_phase) + bias_a
        _dr[i, :] = amplitude * w * np.cos(w * i * dt + bias_phase)
        _ddr[i, :] = -amplitude * w ** 2 * np.sin(w * i * dt + bias_phase)
    return _r, _dr, _ddr


def ref_uav_sequence_Bernoulli(dt: float,
                               tm: float,
                               amplitude: np.ndarray,
                               period: np.ndarray,
                               bias_a: np.ndarray,
                               bias_phase: np.ndarray):
    N = int(np.round(tm / dt))
    _r = np.zeros((N, 4))
    _dr = np.zeros((N, 4))
    _ddr = np.zeros((N, 4))
    w = 2 * np.pi / period
    for i in range(N):
        _r[i, 0] = amplitude[0] * np.cos(w[0] * i * dt + bias_phase[0]) + bias_a[0]
        _r[i, 1] = amplitude[1] * np.sin(2 * w[1] * i * dt + bias_phase[1]) / 2 + bias_a[1]
        _r[i, 2: 4] = amplitude[2: 4] * np.sin(w[2: 4] * i * dt + bias_phase[2: 4]) + bias_a[2: 4]

        _dr[i, 0] = -amplitude[0] * w[0] * np.sin(w[0] * i * dt + bias_phase[0])
        _dr[i, 1] = amplitude[1] * w[1] * np.cos(2 * w[1] * i * dt + bias_phase[1])
        _dr[i, 2: 4] = amplitude[2: 4] * w[2: 4] * np.cos(w[2: 4] * i * dt + bias_phase[2: 4])

        _ddr[i, 0] = -amplitude[0] * w[0] ** 2 * np.cos(w[0] * i * dt + bias_phase[0])
        _ddr[i, 1] = - 2 * amplitude[1] * w[1] ** 2 * np.sin(2 * w[1] * i * dt + bias_phase[1])
        _ddr[i, 2: 4] = -amplitude[2: 4] * w[2: 4] ** 2 * np.sin(w[2: 4] * i * dt + bias_phase[2: 4])

    return _r, _dr, _ddr

def offset_uav_n_sequence(dt: float, tm: float, A: np.ndarray, T: np.ndarray, ba: np.ndarray, bp: np.ndarray):
    N = int(np.round(tm / dt))
    uav_num = A.shape[0]
    _off = np.zeros((N, uav_num, 3))
    _doff = np.zeros((N, uav_num, 3))
    _ddoff = np.zeros((N, uav_num, 3))
    w = 2 * np.pi / T
    for i in range(N):
        _off[i, :, :] = A * np.sin(w * i * dt + bp) + ba
        _doff[i, :, :] = A * w * np.cos(w * i * dt + bp)
        _ddoff[i, :, :] = -A * w ** 2 * np.sin(w * i * dt + bp)
    return _off, _doff, _ddoff
