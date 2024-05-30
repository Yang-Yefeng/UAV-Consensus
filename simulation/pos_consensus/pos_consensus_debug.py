import os
import sys
import datetime
import platform
# import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.BS_FNTSMC import bs_fntsmc, bs_fntsmc_param
from uav.uav_consensus import usv_consensus, uav_param
from utils.ref_cmd import *
from utils.utils import *
from consensus_uncertainty import *

# from utils.collector import data_collector

'''global variables'''
g_dt = 0.01  # global sampling period
g_t = 0.  # global time
g_N = 0  # global step
g_tm = 20  # global maximum simulation time
g_ideal = True  # global ideal
g_obs_in = 'rd3'
g_obs_out = 'rd3'
UAV_NUM = 1
g_A = np.array([[0]]).astype(float)  # 邻接矩阵
g_D = np.array([0]).astype(float)  # 入度矩阵
g_B = np.array([1]).astype(float)  # 通信矩阵
IS_IDEAL = True

cur_time = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M-%S')
cur_path = os.path.dirname(os.path.abspath(__file__))
windows = platform.system().lower() == 'windows'
print(cur_path)
if windows:
    new_path = cur_path + '\\..\\..\\datasave\\pos_consensus-' + cur_time + '/'
else:
    new_path = cur_path + '/../../datasave/pos_consensus-' + cur_time + '/'
'''global variables'''

'''Parameter list of the quadrotor'''
uav_param = uav_param(
    m=0.8,
    g=9.8,
    J=np.array([4.212e-3, 4.212e-3, 8.255e-3]),
    d=0.12,
    CT=2.168e-6,
    CM=2.136e-8,
    J0=1.01e-5,
    kr=1e-3,
    kt=1e-3,
    pos0=np.array([0, 0, 0]),
    vel0=np.array([0, 0, 0]),
    angle0=np.array([0, 0, 0]),
    pqr0=np.array([0, 0, 0]),
    dt=g_dt,
    time_max=g_tm)
'''Parameter list of the quadrotor'''

'''Parameter list of the attitude controller'''
att_ctrl_param = bs_fntsmc_param(
    k1=np.array([5, 5, 5]),  # RL 学
    k2=np.array([8., 8., 20.]),  # RL 学
    k3=np.array([1., 1., 1.5]),  # RL 学
    k4=np.array([0.05, 0.05, 0.05]),
    k5=np.array([5, 5, 5]),  # 要大 RL 学
    alpha1=np.array([1.2, 1.2, 1.5]),
    alpha2=np.array([1.2, 1.2, 1.5]),
    dim=3,
    dt=g_dt
    # k1 控制反步中 wd 的反馈大小
    # k2 控制滑模中 e 的占比，k3 控制滑模中 sig(e)^alpha1 的占比
    # k3-alpha1 的组合不要太大
    # k4 是观测器补偿用的，实际上观测的都很好，所以 k4 要大于0，但是非常小
    # k5-alpha2 的组合要比k3-alpha1 大，但是也别太大
)
'''Parameter list of the attitude controller'''

'''Parameter list of the position controller'''
pos_ctrl_param = bs_fntsmc_param(
    k1=np.array([8, 8, 4]),
    k2=np.array([0.3, 0.3, 1.0]),
    k3=np.array([0.5, 0.5, 1]),
    k4=np.array([0.05, 0.05, 0.05]),        # 补偿观测器的，小点就行
    k5=np.array([6, 6, 6]),
    alpha1=np.array([1.8, 1.8, 1.8]),
    alpha2=np.array([1.01, 1.01, 1.01]),
    dim=3,
    dt=g_dt
)
'''Parameter list of the position controller'''

'''uav group initialization'''
uavs = []
for i in range(UAV_NUM):
    obs_in = rd3(use_freq=True,
                 omega=np.array([3.5, 3.4, 10]),
                 dim=3,
                 thresh=np.array([0.5, 0.5, 0.5]),
                 dt=g_dt)
    obs_in.set_init(all_zero=True)
    obs_out = rd3(use_freq=True,
                  omega=np.array([4, 4, 4]),  # 实际实验这个数一定要小 0.9, 0.9, 0.9，或者从小往大调
                  dim=3,
                  thresh=np.array([0.5, 0.5, 0.5]),
                  dt=g_dt)
    obs_out.set_init(all_zero=True)
    uav = usv_consensus(uav_param=uav_param,
                        ctrl_att_param=att_ctrl_param,
                        ctrl_pos_param=pos_ctrl_param,
                        adjacency=g_A[i],
                        in_degree=g_D[i],
                        communication=g_B[i],
                        obs_att=obs_in,
                        obs_pos=obs_out,
                        is_ideal=IS_IDEAL)
    uavs.append(uav)
'''uav group initialization'''

'''global trajectory'''
ref_amplitude = np.array([4, 4, 2, np.pi / 2])  # x y z psi
# ref_amplitude = np.array([0, 0, 0, 0])  # x y z psi
ref_period = np.array([5, 5, 4, 5])
ref_bias_a = np.array([2, 2, 1, 0])
ref_bias_phase = np.array([np.pi / 2, 0, 0, 0])
'''global trajectory'''

'''local bias'''
r = 0.5
bias = np.array([[0, 0, 0, 0]]).astype(float)
'''local bias'''

'''calculate global eta, dot_eta'''


def cal_g_eta_dot_eta():
    _res = np.zeros((UAV_NUM, 3))
    _dot_res = np.zeros((UAV_NUM, 3))
    for _uav in uavs:
        _res[i][:] = _uav.uav.eta()
        _dot_res[i][:] = _uav.uav.dot_eta()
    return _res, _dot_res


'''calculate global eta, dot_eta'''

'''control'''
if __name__ == '__main__':
    '''1. generate uncertainty for all UAVs'''
    consensus_un = consensus_uncertainty_N(is_ideal=IS_IDEAL, dt=g_dt, tm=g_tm, num_uav=UAV_NUM)  # (20000, 24)
    print(consensus_un.shape)

    while g_t < g_tm - g_dt / 2:
        if g_N % int(1 / g_dt) == 0:
            print('time: %.2f s.' % (g_N / int(1 / g_dt)))

        '''2. calculations for each UAV'''
        for i in range(UAV_NUM):  # 对于每一个无人机
            '''1.1 generate reference command, uncertainty, and bias for each uav'''
            dis_i = consensus_un[g_N, 6 * i: 6 * (i + 1)]
            ref_i, dot_ref_i, _, _ = ref_uav(g_t, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)
            bias_i = bias[i]
            dot_bias_i = np.zeros(4)

            eta_d_i = ref_i[0: 3] + bias_i[0: 3]  # eta 表示外环，d表示参考，i表示无人机编号
            dot_eta_d_i = dot_ref_i[0: 3] + dot_bias_i[0: 3]  # dot 表示一阶导数，eta表示外环，d表示参考，i表示无人机编号

            if not uavs[i].is_ideal:
                syst_dynamic_i = -uavs[i].uav.kt / uavs[i].uav.m * uavs[i].uav.dot_eta() + uavs[i].uav.A()
                obs_eta_i, _ = uavs[i].uav.obs_pos.observe(x=uavs[i].uav.eta(), syst_dynamic=syst_dynamic_i)
            else:
                obs_eta_i = np.zeros(3)

            '''1.2 calculate consensus error'''
            g_eta, g_dot_eta = cal_g_eta_dot_eta()
            uavs[i].ctrl_pos.control_update_outer_consensus(g_eta,
                                                            g_dot_eta,
                                                            eta_d_i,
                                                            dot_eta_d_i,
                                                            uavs[i].uav.eta(),
                                                            uavs[i].uav.dot_eta(),
                                                            uavs[i].d,
                                                            uavs[i].b,
                                                            uavs[i].adjacency,
                                                            uavs[i].uav.kt,
                                                            uavs[i].uav.m,
                                                            obs_eta_i)  # 无人机外环控制
            '''1.3 transfer virtual control command to actual throttle, phi_d, and theta_d'''
            phi_d_old = uavs[i].rho_d[0]
            theta_d_old = uavs[i].rho_d[1]
            phi_d, theta_d, throttle = uo_2_ref_angle_throttle(uo=uavs[i].ctrl_pos.control_out,
                                                               att=uavs[i].uav.uav_att(),
                                                               m=uavs[i].uav.m,
                                                               g=uavs[i].uav.g)

            dot_phi_d = (phi_d - phi_d_old) / uavs[i].uav.dt
            dot_theta_d = (theta_d - theta_d_old) / uavs[i].uav.dt
            uavs[i].rho_d = np.array([phi_d, theta_d, ref_i[3]])  # phi_d theta_d psi_d
            uavs[i].dot_rho_d = np.array([dot_phi_d, dot_theta_d, dot_ref_i[3]])  # phi_d theta_d psi_d 的一阶导数
            uavs[i].throttle = throttle

            '''1.4 inner loop control'''
            e_rho_i = uavs[i].uav.rho1() - uavs[i].rho_d
            de_rho_i = np.dot(uavs[i].uav.W(), uavs[i].uav.rho2()) - uavs[i].dot_rho_d

            if not uavs[i].is_ideal:
                syst_dynamic = (np.dot(uavs[i].uav.dW(), uavs[i].uav.omega()) +
                                np.dot(uavs[i].uav.W(), uavs[i].uav.A_omega() +
                                       np.dot(uavs[i].uav.B_omega(), uavs[i].ctrl_att.control_in)))
                obs_rho_i, _ = uavs[i].obs_att.observe(syst_dynamic=syst_dynamic, x=uavs[i].uav.rho1())
            else:
                obs_rho_i = np.zeros(3)

            # 将观测器输出前两维度置为 0 即可
            obs_rho_i[0] = 0.
            obs_rho_i[1] = 0.
            # 将观测器输出前两维度置为 0 即可
            uavs[i].ctrl_att.control_update_inner(e_rho=e_rho_i,
                                                  de_rho=de_rho_i,
                                                  d_ref=uavs[i].dot_rho_d,
                                                  omega=uavs[i].uav.omega(),
                                                  W=uavs[i].uav.W(),
                                                  dW=uavs[i].uav.dW(),
                                                  A=uavs[i].uav.A_omega(),
                                                  B=uavs[i].uav.B_omega(),
                                                  obs=obs_rho_i)

            '''1.5 rk44 update'''
            action_4_uav_i = np.array([throttle, uavs[i].ctrl_att.control_in[0], uavs[i].ctrl_att.control_in[1], uavs[i].ctrl_att.control_in[2]])
            uavs[i].uav.rk44(action=action_4_uav_i, dis=dis_i, n=1, att_only=False)

            '''1.6 data record'''
            data_block_i = {'time': uavs[i].uav.time,
                            'control': action_4_uav_i,
                            'ref_angle': uavs[i].rho_d,
                            'ref_pos': ref_i[0: 3],
                            'ref_vel': dot_ref_i[0: 3],
                            'd_in': np.array([0., 0., np.dot(uavs[i].uav.W(), np.array([dis_i[3], dis_i[4], dis_i[5]]))[2]]),
                            'd_in_obs': obs_rho_i,
                            'd_in_e_1st': uavs[i].obs_att.obs_error,
                            'd_out': np.array([dis_i[0], dis_i[1], dis_i[2]]) / uavs[i].uav.m,
                            'd_out_obs': obs_eta_i,
                            'd_out_e_1st': uavs[i].obs_pos.obs_error,
                            'state': uavs[i].uav.uav_state_call_back()}
            uavs[i].data_record.record(data=data_block_i)

        g_t += g_dt  # time update
        g_N += 1  # global index update

    SAVE = True
    if SAVE:
        os.mkdir(new_path)
        for i in range(UAV_NUM):
            path = new_path + '\\uav_' + str(i) + '/' if windows else new_path + '/uav_' + str(i) + '/'
            os.mkdir(path)
            uavs[i].data_record.package2file(path)
