import numpy as np


class ftpd_param:
    def __init(self,
               kp_pos: np.ndarray = np.zeros(3),
               ki_pos: np.ndarray = np.zeros(3),
               kd_pos: np.ndarray = np.zeros(3),
               kp_vel: np.ndarray = np.zeros(3),
               ki_vel: np.ndarray = np.zeros(3),
               kd_vel: np.ndarray = np.zeros(3),
               kp_att: np.ndarray = np.zeros(3),
               ki_att: np.ndarray = np.zeros(3),
               kd_att: np.ndarray = np.zeros(3),
               p_v: np.ndarray = np.ones(3),
               p_a: np.ndarray = np.ones(3),
               p_r: np.ndarray = np.ones(3),
               dt: float = 0.01,
               dim:int = 3
               ):
        self.kp_pos = kp_pos
        self.ki_pos = ki_pos
        self.kd_pos = kd_pos

        self.kp_vel = kp_vel
        self.ki_vel = ki_vel
        self.kd_vel = kd_vel

        self.kp_att = kp_att
        self.ki_att = ki_att
        self.kd_att = kd_att

        self.p_v = p_v
        self.p_a = p_a
        self.p_r = p_r
        self.dt = dt
        self.dim = dim


class ftpd(object):
    def __init__(self,
                 param: ftpd_param = None,
                 kp_pos: np.ndarray = np.zeros(3),
                 ki_pos: np.ndarray = np.zeros(3),
                 kd_pos: np.ndarray = np.zeros(3),
                 kp_vel: np.ndarray = np.zeros(3),
                 ki_vel: np.ndarray = np.zeros(3),
                 kd_vel: np.ndarray = np.zeros(3),
                 kp_att: np.ndarray = np.zeros(3),
                 ki_att: np.ndarray = np.zeros(3),
                 kd_att: np.ndarray = np.zeros(3),
                 p_v: np.ndarray = np.ones(3),
                 p_a: np.ndarray = np.ones(3),
                 p_r: np.ndarray = np.ones(3),
                 dt: float = 0.01,
                 dim: int = 3):

        self.kp_pos = kp_pos if param is None else param.kp_pos
        self.ki_pos = ki_pos if param is None else param.ki_pos
        self.kd_pos = kd_pos if param is None else param.kd_pos

        self.kp_vel = kp_vel if param is None else param.kp_vel
        self.ki_vel = ki_vel if param is None else param.ki_vel
        self.kd_vel = kd_vel if param is None else param.kd_vel

        self.kp_att = kp_att if param is None else param.kp_att
        self.ki_att = ki_att if param is None else param.ki_att
        self.kd_att = kd_att if param is None else param.kd_att

        self.p_v = p_v if param is None else param.p_v
        self.p_a = p_a if param is None else param.p_a
        self.p_r = p_r if param is None else param.p_r

        self.dt = dt if param is None else param.dt
        self.dim = dim if param is None else param.dim
        " init control para "

        " simulation state "
        self.err_p_pos = np.zeros(3)
        self.err_i_pos = np.zeros(3)
        self.err_d_pos = np.zeros(3)

        self.err_p_vel = np.zeros(3)
        self.err_i_vel = np.zeros(3)
        self.err_d_vel = np.zeros(3)

        self.err_p_att = np.zeros(3)
        self.err_i_att = np.zeros(3)
        self.err_d_att = np.zeros(3)

        self.control_out_consensus = np.zeros(3)
        self.control = np.zeros(3)

        self.att_control = np.zeros(3)
        " simulation state "

    def para(self):
        print('Para for fix PID',
              'kp_pos:', self.kp_pos,
              'ki_pos:', self.ki_pos,
              'kd_pos:', self.kd_pos,
              'kp_vel:', self.kp_vel,
              'ki_vel:', self.ki_vel,
              'kd_vel:', self.kd_vel,
              'alpha_pos:', self.p_v,
              'alphpos_a:', self.p_a)

    def control_update_outer_consensus(self,
                                       b: float,
                                       d: float,
                                       consensus_e: np.ndarray,
                                       consensus_de: np.ndarray,
                                       dd_ref: np.ndarray,
                                       dd_nu: np.ndarray):
        self.err_p_pos = - consensus_e.clip(np.array([-0.5, -0.5, -1]), np.array([0.5, 0.5, 1]))

        for i in range(3):
            self.err_i_pos[i] = self.err_i_pos[i] * 1.00 + abs(self.err_p_pos[i].clip(-0.3, 0.3)) ** self.p_v[i] * np.sign(40 * self.err_p_pos[i]) * self.dt
            # self.err_i_pos += self.err_p_pos * self.dt

        self.err_d_pos = - consensus_de

        pos_a = np.zeros(3)
        for i in range(3):
            pos_a[i] = self.kp_pos[i] * abs(self.err_p_pos[i]) ** self.p_v[i] * np.tanh(150 * self.err_p_pos[i]) \
                       + self.ki_pos[i] * self.err_i_pos[i] \
                       + self.kd_pos[i] * abs(self.err_d_pos[i]) ** (2 * self.p_v[i] / (1 + self.p_v[i])) * np.tanh(40 * self.err_d_pos[i])

        pos_a = pos_a.clip(np.array([-10, -10, -10]), np.array([10, 10, 10]))
        # pos_a +=  dd_ref
        self.control_out_consensus = 0. * self.control_out_consensus + 1 * pos_a  # low pass filter
        self.control = self.control_out_consensus / (d + b) + b * dd_ref + dd_nu

    def reset(self):
        " simulation state "
        self.err_p_pos = np.zeros(3)
        self.err_i_pos = np.zeros(3)
        self.err_d_pos = np.zeros(3)

        self.err_p_vel = np.zeros(3)
        self.err_i_vel = np.zeros(3)
        self.err_d_vel = np.zeros(3)
        " simulation state "