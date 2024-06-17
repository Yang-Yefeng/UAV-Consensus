import os, sys, datetime, platform, torch
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.FNTSMC import fntsmc, fntsmc_param
from uav.uav import UAV, uav_param
from utils.ref_cmd import *
from utils.utils import *
from utils.collector import data_collector
from utils.PPOActor import PPOActor_Gaussian

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
    k1=np.array([6.00810648, 6.80311651, 13.47563418]).astype(float),  # 手调: 4 4 15
    k2=np.array([2.04587905, 1.60844957, 0.98401018]).astype(float),  # 手调: 1 1 1.5
    k3=np.array([0.05, 0.05, 0.05]).astype(float),
    k4=np.array([9.85776965, 10.91725924, 13.90115023]).astype(float),  # 要大     手调: 5 4 5
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

'''Parameter list of the position controller'''
pos_ctrl_param = fntsmc_param(
    k1=np.array([0.3, 0.3, 1.0]),
    k2=np.array([0.5, 0.5, 1]),
    k3=np.array([0.05, 0.05, 0.05]),  # 补偿观测器的，小点就行
    k4=np.array([6, 6, 6]),
    alpha1=np.array([1.01, 1.01, 1.01]),
    alpha2=np.array([1.01, 1.01, 1.01]),
    dim=3,
    dt=DT
)
'''Parameter list of the position controller'''

def reset_pos_ctrl_param(flag: str):
    if flag == 'zero':
        pos_ctrl_param.k1 = 0.01 * np.ones(3)
        pos_ctrl_param.k2 = 0.01 * np.ones(3)
        pos_ctrl_param.k4 = 0.01 * np.ones(3)
    elif flag == 'random':
        pos_ctrl_param.k1 = np.random.random(3)
        pos_ctrl_param.k2 = np.random.random(3)
        pos_ctrl_param.k4 = np.random.random(3)
    else:  # optimal 手调的
        pos_ctrl_param.k1 = np.array([0.3, 0.3, 1.0])
        pos_ctrl_param.k2 = np.array([0.5, 0.5, 1])
        pos_ctrl_param.k4 = np.array([6, 6, 6])

IS_IDEAL = False

cur_time = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M-%S')
cur_path = os.path.dirname(os.path.abspath(__file__))
windows = platform.system().lower() == 'windows'
if windows:
    new_path = cur_path + '\\..\\..\\datasave\\pos_bs_fntsmc_bs_fntsmc-' + cur_time + '/'
else:
    new_path = cur_path + '/../../datasave/pos_bs_fntsmc_bs_fntsmc-' + cur_time + '/'


if __name__ == '__main__':
    uav = UAV(uav_param)
    obs_in = rd3(use_freq=True, omega=np.array([3.5, 3.4, 6]), dim=3, thresh=np.array([0.5, 0.5, 0.5]), dt=uav.dt)
    obs_out = rd3(use_freq=True, omega=np.array([2, 2, 2]), dim=3, thresh=np.array([0.5, 0.5, 0.5]), dt=uav.dt)
    ctrl_in = fntsmc(att_ctrl_param)
    ctrl_out = fntsmc(pos_ctrl_param)
    
    actor_path = uav.project_path + 'neural_network/pos_maybe_good_3/'
    actor = PPOActor_Gaussian(state_dim=6, action_dim=9)
    actor.load_state_dict(torch.load(actor_path + 'actor'))
    
    uav.load_norm_normalizer_from_file(actor_path + 'state_norm.csv')
    base_path = uav.project_path + 'comparative_cost/position_single/'
    
    A_num = 25
    T_num = 30
    ref_A = np.linspace(0, 2.5, A_num)
    ref_T = np.linspace(5, 8, T_num)
    res = np.zeros((A_num * T_num, 3))
    _i = 0
    
    USE_OBS = False
    USE_RL = False
    for _a in ref_A:
        for _t in ref_T:
            phi_d = theta_d = phi_d_old = theta_d_old = 0.
            dot_phi_d = (phi_d - phi_d_old) / uav.dt
            dot_theta_d = (theta_d - theta_d_old) / uav.dt
            throttle = uav.m * uav.g
            
            uav.reset_with_param()  # 初始化无人机
            obs_in.reset()  # 初始化观测器
            obs_out.reset()
            reset_pos_ctrl_param('optimal')
            ctrl_in.fntsmc_reset_with_new_param(att_ctrl_param)  # 初始化控制器
            ctrl_out.fntsmc_reset_with_new_param(pos_ctrl_param)
            
            while uav.time < uav.time_max - uav.dt / 2:
                # if uav.n % int(1 / uav.dt) == 0:
                #     print('time: %.2f s.' % (uav.n / int(1 / uav.dt)))
                
                '''1. generate reference command and uncertainty'''
                ra = np.array([_a, _a, _a, 0])
                rt = _t * np.ones(4)
                rba = np.zeros(4)
                rbp = np.array([np.pi / 2, 0, 0, 0])
                ref, dot_ref, dotdot_ref = ref_uav(uav.time, ra, rt, rba, rbp)
                uncertainty = generate_uncertainty(time=uav.time, is_ideal=IS_IDEAL)
                
                '''2. generate outer-loop reference signal 'eta_d' and its 1st derivatives'''
                eta_d = ref[0: 3]
                e_eta = uav.eta() - eta_d
                de_eta = uav.dot_eta() - dot_ref[0: 3]
                
                '''3. generate outer-loop virtual control command'''
                if USE_OBS:
                    syst_dynamic = -uav.kt / uav.m * uav.dot_eta() + uav.A()
                    obs_eta, _ = obs_out.observe(x=uav.eta(), syst_dynamic=syst_dynamic)
                else:
                    obs_eta = np.zeros(3)
                
                '''4. outer loop control'''
                if USE_RL:
                    _s = np.concatenate((e_eta, de_eta))
                    new_pos_ctrl_parma = actor.evaluate(uav.state_norm(_s, update=False))
                    hehe = np.array([0, 0, 0, 0, 0, 0, 5, 5, 5]).astype(float)
                    ctrl_out.get_param_from_actor(new_pos_ctrl_parma * hehe, hehe_flag=False)
                ctrl_out.control_update_outer(e_eta=e_eta,
                                              dot_e_eta=de_eta,
                                              dot_eta=uav.dot_eta(),
                                              kt=uav.kt,
                                              m=uav.m,
                                              dd_ref=dotdot_ref[0:3],
                                              obs=obs_eta)
                
                '''5. transfer virtual control command to actual throttle, phi_d, and theta_d'''
                phi_d_old = phi_d
                theta_d_old = theta_d
                phi_d, theta_d, throttle = uo_2_ref_angle_throttle(uo=ctrl_out.control_out, att=uav.uav_att(), m=uav.m, g=uav.g)
                
                dot_phi_d = (phi_d - phi_d_old) / uav.dt
                dot_theta_d = (theta_d - theta_d_old) / uav.dt
                rhod = np.array([phi_d, theta_d, ref[3]])  # phi_d theta_d psi_d
                dot_rhod = np.array([dot_phi_d, dot_theta_d, dot_ref[3]])  # phi_d theta_d psi_d 的一阶导数
                
                uav.sum_reward += uav.get_reward_pos(ref[0: 3], dot_ref[0:3], ctrl_out.control_out)
                
                '''6. inner loop control'''
                e_rho = uav.rho1() - rhod
                de_rho = np.dot(uav.W(), uav.rho2()) - dot_rhod
        
                syst_dynamic = np.dot(uav.dW(), uav.omega()) + np.dot(uav.W(), uav.A_omega() + np.dot(uav.B_omega(), ctrl_in.control_in))
                obs_rho, _ = obs_in.observe(syst_dynamic=syst_dynamic, x=uav.rho1())
                
                obs_rho[0] = 0.
                obs_rho[1] = 0.
                
                ctrl_in.control_update_inner(e_rho=e_rho,
                                             dot_e_rho=de_rho,
                                             dd_ref=np.zeros(3),
                                             W=uav.W(),
                                             dW=uav.dW(),
                                             omega=uav.omega(),
                                             A_omega=uav.A_omega(),
                                             B_omega=uav.B_omega(),
                                             obs=obs_rho,
                                             att_only=False)
                
                '''7. rk44 update'''
                action_4_uav = np.array([throttle, ctrl_in.control_in[0], ctrl_in.control_in[1], ctrl_in.control_in[2]])
                uav.rk44(action=action_4_uav, dis=uncertainty, n=1, att_only=False)
            
            res[_i, :] = np.array([_a, _t, uav.sum_reward])
            print(_i, uav.sum_reward)
            
            _i += 1
    pd.DataFrame(res, columns=['A', 'T', 'r_fntsmc']).to_csv(base_path + 'att_cost_surface_fntsmc_no_obs.csv', sep=',', index=False)
