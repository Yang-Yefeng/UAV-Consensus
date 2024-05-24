import numpy as np


class bs_fntsmc_param:
    def __init__(self,
                 k1: np.ndarray = np.zeros(3),  # backstepping 参数
                 k2: np.ndarray = np.zeros(3),  # 状态误差收敛参数，k2 为 e 的增益
                 k3: np.ndarray = np.zeros(3),  # 状态误差收敛参数，k3 为 sig(e)^\alpha 的增益
                 k4: np.ndarray = np.zeros(3),  # 滑模误差收敛参数，k4 为补偿观测器的增益，理论上是充分小就行
                 k5: np.ndarray = np.zeros(3),  # 滑模误差收敛参数，k5 控制滑模收敛速度
                 alpha1: np.ndarray = 1.5 * np.ones(3),  # 状态误差收敛指数
                 alpha2: np.ndarray = 1.5 * np.ones(3),  # 滑模误差收敛指数
                 dim: int = 3,
                 dt: float = 0.001):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.k4 = k4
        self.k5 = k5
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.dim = dim
        self.dt = dt

    def print_param(self):
        print('==== BackStepping-FNTSMC PARAM ====')
        print('k1:     ', self.k1)
        print('k2:     ', self.k2)
        print('k3:     ', self.k3)
        print('k4:     ', self.k4)
        print('k5:     ', self.k5)
        print('alpha1: ', self.alpha1)
        print('alpha2: ', self.alpha2)
        print('dim:    ', self.dim)
        print('dt:     ', self.dt)
        print('==== BackStepping-FNTSMC PARAM ====')


class bs_fntsmc:
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
        self.k1 = k1 if param is None else param.k1
        self.k2 = k2 if param is None else param.k2
        self.k3 = k3 if param is None else param.k3
        self.k4 = k4 if param is None else param.k4
        self.k5 = k5 if param is None else param.k5
        self.alpha1 = alpha1 if param is None else param.alpha1
        self.alpha2 = alpha2 if param is None else param.alpha2
        self.dim = dim if param is None else param.dim
        self.dt = dt if param is None else param.dt

        self.s = np.zeros(self.dim)
        self.control_in = np.zeros(self.dim)
        self.control_out = np.zeros(self.dim)

    @staticmethod
    def sig(x, a, kt=5):
        return np.fabs(x) ** a * np.tanh(kt * x)

    def bs_inner(self, e_rho: np.ndarray, W: np.ndarray, d_ref: np.ndarray):
        return np.dot(np.linalg.inv(W), -self.k1 * e_rho + d_ref)

    def bs_outer(self, e_eta: np.ndarray, d_ref: np.ndarray):
        return -self.k1 * e_eta + d_ref


    def control_update_inner(self,
                             e_rho: np.ndarray,
                             de_rho: np.ndarray,
                             d_ref: np.ndarray,
                             omega: np.ndarray,
                             W: np.ndarray,
                             dW: np.ndarray,
                             A: np.ndarray,
                             B: np.ndarray,
                             obs: np.ndarray):
        """
        :param e_rho:		内环控制误差
        :param de_rho:		内环控制误差导数
        :param d_ref:		内环参考信息一阶导
        :param omega:		角速度
        :param W:			坐标转换矩阵
        :param dW:			W 的导数
        :param A:			等效动态矩阵
        :param B:			等效系统矩阵
        :param obs:			观测器
        :return:			无 (控制量)
        """
        '''计算滑模'''
        self.s = de_rho + self.k2 * e_rho + self.k3 * self.sig(e_rho, self.alpha1)
        '''计算滑模'''
        omega_d = self.bs_inner(e_rho, W, d_ref)

        yyf1 = self.k1 ** 2 * np.dot(W, np.linalg.inv(W).T) + self.k1 * np.dot(W, np.dot(np.linalg.inv(W).T, W))
        f_rho1 = np.dot(yyf1, e_rho)

        f_rho2 = self.k2 * de_rho + self.k3 * self.alpha1 * self.sig(e_rho, self.alpha1 - 1) * de_rho

        e_omega = omega - omega_d
        f_rho3 = np.dot(dW, e_omega) + np.dot(W, A)

        f_rho = f_rho1 + f_rho2 + f_rho3

        tau1 = -f_rho
        # tau2 = -np.dot(W, obs)
        tau2 = -obs
        tau3 = -self.k4 * np.tanh(5 * self.s)
        tau4 = -self.k5 * self.sig(self.s, self.alpha2)
        self.control_in = np.dot(np.linalg.inv(np.dot(W, B)), tau1 + tau2 + tau3 + tau4)

    def control_update_outer(self,
                             kt: float,
                             m: float,
                             dot_eta: np.ndarray,
                             e_eta: np.ndarray,
                             d_ref: np.ndarray,
                             obs: np.ndarray):
        ev = dot_eta - self.bs_outer(e_eta, d_ref)
        de_eta = -self.k1 * e_eta + ev

        self.s = de_eta + self.k2 * e_eta + self.k3 * self.sig(e_eta, self.alpha1)

        u1 = kt / m * dot_eta
        u2 = -self.k2 * de_eta - self.k3 * self.alpha1 * self.sig(e_eta, self.alpha1 - 1) * de_eta
        u3 = -obs - self.k4 * np.tanh(5 * self.s) - self.k5 * self.sig(self.s, self.alpha2)

        self.control_out = u1 + u2 + u3
