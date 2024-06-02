import os
import sys
import datetime
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import platform

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.BS_FNTSMC import bs_fntsmc, bs_fntsmc_param
from uav.uav import UAV, uav_param
from utils.ref_cmd import *
from utils.utils import *
from utils.collector import data_collector

cur_path = os.path.dirname(os.path.abspath(__file__))
windows = platform.system().lower() == 'windows'

'''Parameter list of the quadrotor'''
DT = 0.01
uav_param = uav_param()
uav_param.m = 0.8
uav_param.g = 9.8
uav_param.J = np.array([4.212e-3, 4.212e-3, 8.255e-3])
uav_param.d = 0.12
uav_param.CT = 2.168e-6
uav_param.CM = 2.136e-8
uav_param.J0 = 1.01e-5
uav_param.kr = 1e-3
uav_param.kt = 1e-3
uav_param.pos0 = np.array([0, 0, 0])
uav_param.vel0 = np.array([0, 0, 0])
uav_param.angle0 = np.array([0, 0, 0])
uav_param.pqr0 = np.array([0, 0, 0])
uav_param.dt = DT
uav_param.time_max = 20
'''Parameter list of the quadrotor'''

'''Parameter list of the attitude controller'''
att_ctrl_param = bs_fntsmc_param(
    k1=np.array([3, 3, 3]),
    k2=np.array([8., 8., 20.]),
    k3=np.array([1., 1., 1.5]),
    k4=np.array([0.05, 0.05, 0.05]),
    k5=np.array([5, 4, 5]),       # 要大
    alpha1=np.array([1.01, 1.01, 1.01]),
    alpha2=np.array([1.01, 1.01, 1.01]),
    dim=3,
    dt=DT
    # k1 控制反步中 wd 的反馈大小
    # k2 控制滑模中 e 的占比，k3 控制滑模中 sig(e)^alpha1 的占比
    # k3-alpha1 的组合不要太大
    # k4 是观测器补偿用的，实际上观测的都很好，所以 k4 要大于0，但是非常小
    # k5-alpha2 的组合要比k3-alpha1 大，但是也别太大
)
'''Parameter list of the attitude controller'''

IS_IDEAL = False
OBSERVER = 'rd3'

if __name__ == '__main__':
    uav = UAV(uav_param)
    ctrl_in = bs_fntsmc(att_ctrl_param)

    ref_amplitude = np.array([deg2rad(70), deg2rad(70), np.pi / 2])
    ref_period = np.array([5, 5, 4])
    ref_bias_a = np.array([0, 0, 0])
    ref_bias_phase = np.array([0, np.pi / 2, 0])

    rhod, dot_rhod, dot2_rhod, dot3_rhod = ref_inner(uav.time, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)
    e0 = uav.rho1() - rhod
    de0 = uav.dot_rho1() - dot_rhod

    if OBSERVER == 'rd3':
        '''
            m 和 n 可以相等，也可以不同。m对应低次，n对应高次。
        '''
        observer = rd3(use_freq=True,
                       omega=np.array([7, 7, 7]),
                       dim=3,
                       dt=uav.dt)
        syst_dynamic0 = np.dot(uav.dW(), uav.omega()) + np.dot(uav.W(), uav.A_omega() + np.dot(uav.B_omega(), ctrl_in.control_in))
        observer.set_init(e0=e0, de0=de0, syst_dynamic=syst_dynamic0)
    else:
        observer = None

    de = np.array([0, 0, 0]).astype(float)
    data_record = data_collector(N=int(uav.time_max / uav.dt))

    while uav.time < uav.time_max:
        if uav.n % int(1 / uav.dt) == 0:
            print('time: %.2f s.' % (uav.n / int(1 / uav.dt)))

        '''1. 计算 tk 时刻参考信号 和 生成不确定性'''
        uncertainty = generate_uncertainty(time=uav.time, is_ideal=IS_IDEAL)
        rhod, dot_rhod, dot2_rhod, dot3_rhod = ref_inner(uav.time, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)
        '''1. 计算 tk 时刻参考信号 和 生成不确定性'''

        '''2. 计算 tk 时刻误差信号'''
        e_rho = uav.rho1() - rhod
        de_rho = uav.dot_rho1() - dot_rhod
        '''2. 计算 tk 时刻误差信号'''

        '''3. 观测器'''
        if not IS_IDEAL:
            syst_dynamic = np.dot(uav.dW(), uav.omega()) + np.dot(uav.W(), uav.A_omega() + np.dot(uav.B_omega(), ctrl_in.control_in))
            delta_obs, dot_delta_obs = observer.observe(syst_dynamic=syst_dynamic, x=uav.rho1())
        else:
            delta_obs, dot_delta_obs = np.zeros(3), np.zeros(3)
        '''3. 观测器'''

        '''4. 计算控制量'''
        ctrl_in.control_update_inner(e_rho=e_rho,
                                     de_rho=de_rho,
                                     d_ref=dot_rhod,
                                     omega=uav.omega(),
                                     W=uav.W(),
                                     dW=uav.dW(),
                                     A=uav.A_omega(),
                                     B=uav.B_omega(),
                                     obs=delta_obs)
        '''4. 计算控制量'''

        '''5. 状态更新'''
        action_4_uav = np.array([uav.m * uav.g, ctrl_in.control_in[0], ctrl_in.control_in[1], ctrl_in.control_in[2]])
        uav.rk44(action=action_4_uav, dis=uncertainty, n=1, att_only=True)
        '''5. 状态更新'''

        '''6. 数据存储'''
        if IS_IDEAL:
            in_obs_error = np.zeros(3)
        else:
            in_obs_error = observer.obs_error
        data_block = {'time': uav.time,
                      'control': action_4_uav,
                      'ref_angle': rhod,
                      'ref_pos': np.array([0., 0., 0.]),
                      'ref_vel': np.array([0., 0., 0.]),
                      'd_in': np.dot(uav.W(), np.array([uncertainty[3], uncertainty[4], uncertainty[5]])),
                      'd_in_obs': delta_obs,
                      'd_in_e_1st': in_obs_error,
                      # 'd_in': np.array([uncertainty[3], uncertainty[4], uncertainty[5]]),
                      # 'd_in_obs': np.dot(np.linalg.inv(uav.W()), delta_obs),
                      'd_out': np.array([uncertainty[0], uncertainty[1], uncertainty[2]]) / uav.m,
                      'd_out_obs': np.array([0., 0., 0.]),
                      'd_out_e_1st': np.zeros(3),
                      'state': np.hstack((np.zeros(6), uav.uav_att_pqr_call_back()))}
        data_record.record(data=data_block)
        '''6. 数据存储'''

    SAVE = True
    if SAVE:
        if windows:
            new_path = cur_path + '\\..\\datasave\\att_bs_fntsmc-' + datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M-%S') + '/'
        else:
            new_path = cur_path + '/../datasave/att_bs_fntsmc-' + datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M-%S') + '/'
        os.mkdir(new_path)
        data_record.package2file(new_path)

    data_record.plot_att()
    data_record.plot_torque()
    data_record.plot_inner_obs()
    plt.show()
