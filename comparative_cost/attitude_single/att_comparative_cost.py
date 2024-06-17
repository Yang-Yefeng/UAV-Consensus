import os
import sys
import datetime
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import platform

import torch

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.FNTSMC import fntsmc_param, fntsmc
from uav.uav import UAV, uav_param
from utils.ref_cmd import *
from utils.utils import *
from utils.PPOActor import PPOActor_Gaussian

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
uav_param.time_max = 10
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


def reset_att_ctrl_param(flag: str):
    if flag == 'zero':
        att_ctrl_param.k1 = 0.01 * np.ones(3)
        att_ctrl_param.k2 = 0.01 * np.ones(3)
        att_ctrl_param.k4 = 0.01 * np.ones(3)
    elif flag == 'random':
        att_ctrl_param.k1 = np.random.random(3)
        att_ctrl_param.k2 = np.random.random(3)
        att_ctrl_param.k4 = np.random.random(3)
    else:  # optimal 手调的
        att_ctrl_param.k1 = np.array([4., 4., 15.])
        att_ctrl_param.k2 = np.array([1., 1., 1.5])
        att_ctrl_param.k4 = np.array([5, 4, 5])


IS_IDEAL = True

# RL
USE_OBS = False
USE_RL = True


def AoLiGeiGanLe(use_obs:bool, use_rl:bool) -> np.ndarray:
    A_num = 10 # 31
    T_num = 5  # 15
    ref_A = np.linspace(deg2rad(10), deg2rad(70), A_num)
    ref_T = np.linspace(3, 6, T_num)
    res = np.zeros((A_num * T_num, 3))
    _i = 0
    for _a in ref_A:
        for _t in ref_T:
            ra = _a * np.ones(3)
            rp = _t * np.ones(3)
            rba = np.array([0, 0, 0])
            rbp = np.array([0, 0, 0])
            rho_d_all, dot_rho_d_all, dot2_rho_d_all = ref_inner_all(ra, rp, rba, rbp, uav.time_max, uav.dt)
            
            uav.reset_with_param()  # 初始化无人机
            observer.reset()  # 初始化观测器
            ctrl_in.fntsmc_reset_with_new_param(att_ctrl_param)  # 初始化控制器
            
            while uav.time < uav.time_max:
                '''1. 计算 tk 时刻参考信号 和 生成不确定性'''
                uncertainty = generate_uncertainty(time=uav.time, is_ideal=IS_IDEAL)
                rho_d, dot_rho_d, dot2_rho_d = rho_d_all[uav.n], dot_rho_d_all[uav.n], dot2_rho_d_all[uav.n]
                
                '''2. 计算 tk 时刻误差信号'''
                e_rho = uav.rho1() - rho_d
                de_rho = uav.dot_rho1() - dot_rho_d
                
                '''3. 观测器'''
                if use_obs:
                    syst_dynamic = np.dot(uav.dW(), uav.omega()) + np.dot(uav.W(), uav.A_omega() + np.dot(uav.B_omega(), ctrl_in.control_in))
                    delta_obs, _ = observer.observe(syst_dynamic=syst_dynamic, x=uav.rho1())
                else:
                    delta_obs = np.zeros(3)
                
                '''4. 计算控制量'''
                if use_rl:
                    _s = np.concatenate((e_rho, de_rho))
                    new_att_ctrl_parma = actor.evaluate(uav.state_norm(_s, update=False))
                    hehe = np.array([5, 5, 5, 0, 0, 0, 5, 5, 5]).astype(float)
                    ctrl_in.get_param_from_actor(new_att_ctrl_parma * hehe, hehe_flag=False)
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
                uav.sum_reward += uav.get_reward_att(rho_d, dot_rho_d, ctrl_in.control_in)
            
            res[_i, :] = np.array([_a, _t, uav.sum_reward])
            if _i % 50 == 0:
                print('...', _i, '...')
            _i += 1
    
    return res

if __name__ == '__main__':
    uav = UAV(uav_param)
    observer = rd3(use_freq=True, omega=np.array([6, 6, 6]), dim=3, dt=uav.dt)
    ctrl_in = fntsmc(att_ctrl_param)
    
    actor_path = uav.project_path + 'neural_network/att_maybe_good_1/'
    actor = PPOActor_Gaussian(state_dim=6, action_dim=9)
    actor.load_state_dict(torch.load(actor_path + 'actor'))
    
    uav.load_norm_normalizer_from_file(actor_path + 'state_norm.csv')
    base_path = uav.project_path + 'comparative_cost/attitude_single/'
    
    # print('Group1')
    # r1 = AoLiGeiGanLe(use_obs=False, use_rl=True)
    # pd.DataFrame(r1, columns=['A', 'T', 'r_rl']).to_csv(base_path + 'att_cost_surface_rl_no_obs.csv', sep=',', index=False)
    
    # print('Group2')
    # r2 = AoLiGeiGanLe(use_obs=True, use_rl=True)
    # pd.DataFrame(r2, columns=['A', 'T', 'r_rl_obs']).to_csv(base_path + 'att_cost_surface_rl_obs.csv', sep=',', index=False)

    print('Group3')
    r3 = AoLiGeiGanLe(use_obs=False, use_rl=False)
    pd.DataFrame(r3, columns=['A', 'T', 'r_fntsmc']).to_csv(base_path + 'att_cost_surface_fntmsc_no_obs.csv', sep=',', index=False)

    # print('Group4')
    # r4 = AoLiGeiGanLe(use_obs=True, use_rl=False)
    # pd.DataFrame(r4, columns=['A', 'T', 'r_fntsmc_obs']).to_csv(base_path + 'att_cost_surface_fntmsc_obs.csv', sep=',', index=False)
    
    # cost = np.concatenate((r1[:, 0: 3], r2[:, 2, None], r3[:, 2, None], r4[:, 2, None]), axis=1)
    # pd.DataFrame(cost, columns=['A', 'T', 'r_rl', 'r_rl_obs', 'r_fntsmc', 'r_fntsmc_obs']).to_csv('att_cost_surface.csv', sep=',', index=False)
