import os
import sys
import datetime
import matplotlib.pyplot as plt
import platform

import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.BS_FNTSMC import bs_fntsmc, bs_fntsmc_param
from uav.uav import UAV, uav_param
from utils.ref_cmd import *
from utils.utils import *
from utils.collector import data_collector

'''Parameter list of the quadrotor'''
DT = 0.001
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
    # k1=np.array([7, 7, 7]),
    # k2=np.array([4., 4., 6.]),
    # k3=np.array([1.5, 1.5, 1.5]),
    # k4=np.array([1e-1, 1e-1, 1e-1]),
    # k5=np.array([15, 15, 25]),  # 要大
    # alpha1=np.array([1.5, 1.5, 1.5]),
    # alpha2=np.array([2, 2, 2]),
    k1=np.array([5, 5, 5]),
    k2=np.array([8., 8., 20.]),
    k3=np.array([1., 1., 1.5]),
    k4=np.array([0.05, 0.05, 0.05]),
    k5=np.array([5, 5, 5]),  # 要大
    alpha1=np.array([1.2, 1.2, 1.5]),
    alpha2=np.array([1.2, 1.2, 1.5]),
    dim=3,
    dt=DT
    # k1 控制反步中 wd 的反馈大小
    # k2 控制滑模中 e 的占比，k3 控制滑模中 sig(e)^alpha1 的占比
    # k3-alpha1 的组合不要太大
    # k4 是观测器补偿用的，实际上观测的都很好，所以 k4 要大于0，但是非常小
    # k5-alpha2 的组合要比k3-alpha1 大，但是也别太大
)
'''Parameter list of the attitude controller'''

'''Parameter list of the position controller'''
pos_ctrl_param = bs_fntsmc_param(
    # k1=np.array([6, 6, 4]),
    # k2=np.array([0.3, 0.3, 1.0]),
    # k3=np.array([0.5, 0.5, 1]),
    # k4=np.array([0.05, 0.05, 0.05]),
    # k5=np.array([8, 8, 8]),
    # alpha1=np.array([1.2, 1.2, 1.5]),
    # alpha2=np.array([1.2, 1.2, 1.2]),
    # dim=3,
    # dt=DT
    k1=np.array([8, 8, 4]),
    k2=np.array([0.3, 0.3, 1.0]),
    k3=np.array([0.5, 0.5, 1]),
    k4=np.array([0.05, 0.05, 0.05]),        # 补偿观测器的，小点就行
    k5=np.array([6, 6, 6]),
    alpha1=np.array([1.8, 1.8, 1.8]),
    alpha2=np.array([1.01, 1.01, 1.01]),
    dim=3,
    dt=DT
)
'''Parameter list of the position controller'''

IS_IDEAL = False
OBSERVER_IN = 'rd3'
OBSERVER_OUT = 'rd3'

cur_time = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M-%S')
cur_path = os.path.dirname(os.path.abspath(__file__))
windows = platform.system().lower() == 'windows'
if windows:
    new_path = cur_path + '\\..\\datasave\\pos_bs_fntsmc_bs_fntsmc-' + cur_time + '/'
else:
    new_path = cur_path + '/../datasave/pos_bs_fntsmc_bs_fntsmc-' + cur_time + '/'

if __name__ == '__main__':
    uav = UAV(uav_param)
    ctrl_in = bs_fntsmc(att_ctrl_param)
    ctrl_out = bs_fntsmc(pos_ctrl_param)

    '''reference signal initialization'''
    ref_amplitude = np.array([2, 2, 1, np.pi / 2])  # x y z psi
    # ref_amplitude = np.array([0, 0, 0, 0])  # x y z psi
    ref_period = np.array([5, 5, 4, 5])
    ref_bias_a = np.array([2, 2, 1, 0])
    ref_bias_phase = np.array([np.pi / 2, 0, 0, 0])

    '''data storage initialization'''
    data_record = data_collector(N=int(uav.time_max / uav.dt))

    '''reference signal'''
    phi_d = theta_d = phi_d_old = theta_d_old = 0.
    dot_phi_d = (phi_d - phi_d_old) / uav.dt
    dot_theta_d = (theta_d - theta_d_old) / uav.dt
    throttle = uav.m * uav.g

    ref, dot_ref, dot2_ref, _ = ref_uav(uav.time, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)  # 整体参考信号 xd yd zd psid
    rhod = np.array([phi_d, theta_d, ref[3]]).astype(float)  # 内环参考信号 phi_d theta_d psi_d
    dot_rhod = np.array([dot_phi_d, dot_theta_d, dot_ref[3]]).astype(float)  # 内环参考信号导数

    '''initial error'''
    e_rho = uav.rho1() - rhod
    de_rho = uav.dot_rho1() - dot_rhod
    e_eta = uav.eta() - ref[0: 3]
    de_eta = uav.dot_eta() - dot_ref[0: 3]

    '''observer'''
    if not IS_IDEAL:
        obs_in = rd3(use_freq=True,
                     omega=np.array([3.5, 3.4, 10]),
                     dim=3,
                     thresh=np.array([0.5, 0.5, 0.5]),
                     dt=uav.dt)
        syst_dyin = np.dot(uav.dW(), uav.omega()) + np.dot(uav.W(), uav.A_omega() + np.dot(uav.B_omega(), ctrl_in.control_in))
        obs_in.set_init(e0=e_rho, de0=de_rho, syst_dynamic=syst_dyin)

        obs_out = rd3(use_freq=True,
                      omega=np.array([4, 4, 4]),  # 实际实验这个数一定要小 0.9, 0.9, 0.9，或者从小往大调
                      dim=3,
                      thresh=np.array([0.5, 0.5, 0.5]),
                      dt=uav.dt)
        syst_dyout = -uav.kt / uav.m * uav.dot_eta() + uav.A()
        obs_out.set_init(e0=uav.eta(), de0=uav.dot_eta(), syst_dynamic=syst_dyout)
    else:
        obs_in = None
        obs_out = None

    '''control'''
    while uav.time < uav.time_max - uav.dt / 2:
        if uav.n % int(1 / uav.dt) == 0:
            print('time: %.2f s.' % (uav.n / int(1 / uav.dt)))

        '''1. generate reference command and uncertainty'''
        ref, dot_ref, _, _ = ref_uav(uav.time, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)
        uncertainty = generate_uncertainty(time=uav.time, is_ideal=IS_IDEAL)

        '''2. generate outer-loop reference signal 'eta_d' and its 1st derivatives'''
        eta_d = ref[0: 3]
        e_eta = uav.eta() - eta_d
        de_eta = uav.dot_eta() - dot_ref[0: 3]

        '''3. generate outer-loop virtual control command'''
        if not IS_IDEAL:
            syst_dynamic = -uav.kt / uav.m * uav.dot_eta() + uav.A()
            obs_eta, _ = obs_out.observe(x=uav.eta(), syst_dynamic=syst_dynamic)
        else:
            obs_eta = np.zeros(3)

        '''4. outer loop control'''
        ctrl_out.control_update_outer(kt=uav.kt,
                                      m=uav.m,
                                      dot_eta=uav.dot_eta(),
                                      e_eta=e_eta,
                                      d_ref=dot_ref[0: 3],
                                      obs=obs_eta)

        '''5. transfer virtual control command to actual throttle, phi_d, and theta_d'''
        phi_d_old = phi_d
        theta_d_old = theta_d
        phi_d, theta_d, throttle = uo_2_ref_angle_throttle(uo=ctrl_out.control_out, att=uav.uav_att(), m=uav.m, g=uav.g)

        dot_phi_d = (phi_d - phi_d_old) / uav.dt
        dot_theta_d = (theta_d - theta_d_old) / uav.dt
        rhod = np.array([phi_d, theta_d, ref[3]])  # phi_d theta_d psi_d
        dot_rhod = np.array([dot_phi_d, dot_theta_d, dot_ref[3]])  # phi_d theta_d psi_d 的一阶导数

        '''6. inner loop control'''
        e_rho = uav.rho1() - rhod
        de_rho = np.dot(uav.W(), uav.rho2()) - dot_rhod

        if not IS_IDEAL:
            syst_dynamic = np.dot(uav.dW(), uav.omega()) + np.dot(uav.W(), uav.A_omega() + np.dot(uav.B_omega(), ctrl_in.control_in))
            obs_rho, _ = obs_in.observe(syst_dynamic=syst_dynamic, x=uav.rho1())
        else:
            obs_rho = np.zeros(3)

        # 将观测器输出前两维度置为 0 即可
        obs_rho[0] = 0.
        obs_rho[1] = 0.
        # 将观测器输出前两维度置为 0 即可

        ctrl_in.control_update_inner(e_rho=e_rho,
                                     de_rho=de_rho,
                                     d_ref=dot_rhod,
                                     omega=uav.omega(),
                                     W=uav.W(),
                                     dW=uav.dW(),
                                     A=uav.A_omega(),
                                     B=uav.B_omega(),
                                     obs=obs_rho)

        '''7. rk44 update'''
        action_4_uav = np.array([throttle, ctrl_in.control_in[0], ctrl_in.control_in[1], ctrl_in.control_in[2]])
        uav.rk44(action=action_4_uav, dis=uncertainty, n=1, att_only=False)

        '''8. data record'''
        if IS_IDEAL:
            in_obs_error = np.zeros(3)
            out_obs_error = np.zeros(3)
        else:
            in_obs_error = obs_in.obs_error
            out_obs_error = obs_out.obs_error
        data_block = {'time': uav.time,
                      'control': action_4_uav,
                      'ref_angle': rhod,
                      'ref_pos': ref[0: 3],
                      'ref_vel': dot_ref[0: 3],
                      'd_in': np.array([0., 0., np.dot(uav.W(), np.array([uncertainty[3], uncertainty[4], uncertainty[5]]))[2]]),
                      'd_in_obs': obs_rho,
                      'd_in_e_1st': in_obs_error,
                      'd_out': np.array([uncertainty[0], uncertainty[1], uncertainty[2]]) / uav.m,
                      'd_out_obs': obs_eta,
                      'd_out_e_1st': out_obs_error,
                      'state': uav.uav_state_call_back()}
        data_record.record(data=data_block)

    SAVE = False
    if SAVE:
        os.mkdir(new_path)
        data_record.package2file(new_path)

    data_record.plot_att()
    data_record.plot_vel()
    data_record.plot_pos()
    data_record.plot_throttle()
    data_record.plot_torque()
    data_record.plot_inner_obs()
    data_record.plot_outer_obs()
    # data_record.plot_outer_obs()
    # data_record.plot_inner_obs()
    plt.show()
