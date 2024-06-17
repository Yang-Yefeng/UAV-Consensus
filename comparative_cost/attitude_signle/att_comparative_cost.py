import os
import sys
import datetime
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import platform

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.FNTSMC import fntsmc_param, fntsmc
from uav.uav import UAV, uav_param
from utils.ref_cmd import *
from utils.utils import *

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
att_ctrl_param = fntsmc_param(
    k1=np.array([4., 4., 15.]).astype(float),
    k2=np.array([1., 1., 1.5]).astype(float),
    k3=np.array([0.05, 0.05, 0.05]).astype(float),
    k4=np.array([5, 4, 5]).astype(float),  # 要大
    alpha1=np.array([1.01, 1.01, 1.01]).astype(float),
    alpha2=np.array([1.01, 1.01, 1.01]).astype(float),
    dim=3,
    dt=DT
    # k1 控制滑模中 e 的占比，k3 控制滑模中 sig(e)^alpha1 的占比
    # k2-alpha1 的组合不要太大
    # k3 是观测器补偿用的，实际上观测的都很好，所以 k4 要大于0，但是非常小
    # k4-alpha2 的组合要比k3-alpha1 大，但是也别太大
)
'''Parameter list of the attitude controller'''

IS_IDEAL = False
USE_OBS = True
USE_RL = False


if __name__ == '__main__':
    uav = UAV(uav_param)
    observer = rd3(use_freq=True, omega=np.array([7, 7, 7]), dim=3, dt=uav.dt)
    ctrl_in = fntsmc(att_ctrl_param)
    
    A_num = 61
    T_num = 15
    ref_A = np.linspace(0, deg2rad(60), A_num)
    ref_T = np.linspace(3, 6, T_num)
    
    cost = np.zeros((A_num * T_num, 6))  # a, t, r1 ,r2
    
    for _a in ref_A:
        for _t in ref_T:
            uav.reset_with_param()  # 初始化无人机
            observer.reset()  # 初始化观测器
            ctrl_in.reset()  # 初始化控制器
            
            ref_amplitude = _a * np.ones(3)
            ref_period = _t * np.ones(3)
            ref_bias_a = np.array([0, 0, 0])
            ref_bias_phase = np.array([0, 0, 0])
            rho_d_all, dot_rho_d_all, dot2_rho_d_all = ref_inner_all(ref_amplitude, ref_period, ref_bias_a, ref_bias_phase, uav.time_max, uav.dt)
            
            while uav.time < uav.time_max:
                '''1. 计算 tk 时刻参考信号 和 生成不确定性'''
                uncertainty = generate_uncertainty(time=uav.time, is_ideal=IS_IDEAL)
                rho_d, dot_rho_d, dot2_rho_d = rho_d_all[uav.n], dot_rho_d_all[uav.n], dot2_rho_d_all[uav.n]
                
                '''2. 计算 tk 时刻误差信号'''
                e_rho = uav.rho1() - rho_d
                de_rho = uav.dot_rho1() - dot_rho_d
                
                '''3. 观测器'''
                if USE_OBS:
                    syst_dynamic = np.dot(uav.dW(), uav.omega()) + np.dot(uav.W(), uav.A_omega() + np.dot(uav.B_omega(), ctrl_in.control_in))
                    delta_obs, _ = observer.observe(syst_dynamic=syst_dynamic, x=uav.rho1())
                else:
                    delta_obs = np.zeros(3)
                
                '''4. 计算控制量'''
                ctrl_in.control_update_inner(e_rho=e_rho,
                                             dot_e_rho=de_rho,
                                             dd_ref=dot2_rho_d,
                                             W=uav.W(),
                                             dW=uav.dW(),
                                             omega=uav.omega(),
                                             A_omega=uav.A_omega(),
                                             B_omega=uav.B_omega(),
                                             obs=delta_obs,
                                             att_only=True)
                
                '''5. 状态更新'''
                action_4_uav = np.array([uav.m * uav.g, ctrl_in.control_in[0], ctrl_in.control_in[1], ctrl_in.control_in[2]])
                uav.rk44(action=action_4_uav, dis=uncertainty, n=1, att_only=True)
                
                '''6. 计算惩罚'''
                uav.sum_reward += uav.get_reward(rho_d, dot_rho_d, ctrl_in.control_in)
                
                '''7. 数据存储'''
                
                
    pd.DataFrame(cost, columns=['A', 'T', 'r_rl_obs', 'r_rl', 'r_fntsmc_obs', 'r_fntsmc']).to_csv('att_cost_surface.csv', sep=',', index=False)
