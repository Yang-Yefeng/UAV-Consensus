import os
import sys
import datetime
import platform

import matplotlib.pyplot as plt
import numpy as np

# import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
# from controller.BS_FNTSMC import bs_fntsmc_param
from uav.uav_consensus import usv_consensus, uav_param
from utils.ref_cmd import *
from utils.utils import *
from consensus_uncertainty import *

cur_time = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d-%H-%M-%S')
cur_path = os.path.dirname(os.path.abspath(__file__))
windows = platform.system().lower() == 'windows'
print(cur_path)
if windows:
    new_path = cur_path + '\\..\\..\\datasave\\pos_consensus-' + cur_time + '/'
else:
    new_path = cur_path + '/../../datasave/pos_consensus-' + cur_time + '/'

config_file = cur_path + '/global_configuration.xml'
config_root = XML_Load(config_file)  # 加载全局配置文件
g_v = get_global_variable_from_XML(config_root)  # 加载一些全局变量
uav_par = get_uav_param_from_XML(config_root)
uav_par['dt'] = g_v['dt']  # 仅仅是为了防止采样周期不一样，全部以global variable为准
uav_par['g_tm'] = g_v['g_tm']  # 仅仅是为了防止采样周期不一样，全部以global variable为准
uav_par = uav_param(from_dict=uav_par)
att_ctrl_param = get_att_ctrl_parma_from_XML(config_root)
att_ctrl_param.dt = g_v['dt']
'''
对于控制器参数调节:
1. k1 k2 k3 k5 是用 RL 学的
2. k1 控制反步中 wd 的反馈大小
3. k2 控制滑模中 e 的占比，k3 控制滑模中 sig(e)^alpha1 的占比
4. k3-alpha1 的组合不要太大
5. k4 是观测器补偿用的，实际上观测的都很好，所以 k4 要大于0，但是非常小
6. k5-alpha2 的组合要比 k3-alpha1 大，但是也别太大
'''
pos_ctrl_param = get_pos_ctrl_parma_from_XML(config_root)
pos_ctrl_param.dt = g_v['dt']

'''global center trajectory and offset'''
# 第一组
# ref_amplitude = np.array([1, 1, 0.5, np.pi / 2])  # x y z psi
# ref_period = np.array([5, 5, 4, 5])
# ref_bias_a = np.array([0, 0, 1.0, 0])
# ref_bias_phase = np.array([np.pi / 2, 0, 0, 0])
#
# offset_amplitude = np.array([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
# offset_period = np.array([[5, 5, 4], [5, 5, 4], [5, 5, 4], [5, 5, 4]])
# offset_bias_a = np.array([[0.5, 0, 0], [0, 0.5, 0], [-0.5, 0., 0.], [0., -0.5, 0.]])
# offset_bias_phase = np.array([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
# 第一组

# 第二组
# ref_amplitude = np.array([0, 0, 0, 0])  # x y z psi
# ref_period = np.array([5, 5, 4, 5])
# ref_bias_a = np.array([2, 2, 1.0, 0])
# ref_bias_phase = np.array([np.pi / 2, 0, 0, 0])
#
# offset_amplitude = np.array([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
# offset_period = np.array([[5, 5, 4], [5, 5, 4], [5, 5, 4], [5, 5, 4]])
# offset_bias_a = np.array([[0.5, 0, 0], [0, 0.5, 0], [-0.5, 0., 0.], [0., -0.5, 0.]])
# offset_bias_phase = np.array([[0., 0., 0.], [0., 0., 0.],[0., 0., 0.],[0., 0., 0.]])
# 第二组

# 第三组
ref_amplitude = np.array([1, 1, 0.5, np.pi / 2])  # x y z psi
ref_period = np.array([10, 10, 8, 10])
ref_bias_a = np.array([0, 0, 1.0, 0])
ref_bias_phase = np.array([np.pi / 2, 0, 0, 0])

offset_amplitude = np.array([[0.5, 0.5, 0.], [0.5, 0.5, 0.], [0.5, 0.5, 0.], [0.5, 0.5, 0.]])
offset_period = np.array([[5, 5, 4], [5, 5, 4], [5, 5, 4], [5, 5, 4]])
offset_bias_a = np.array([[0., 0., 0], [0., 0., 0], [0., 0., 0.], [0., 0., 0.]])
offset_bias_phase = np.array([[np.pi / 2, 0., 0.], [np.pi, np.pi / 2, 0.],[-np.pi / 2, np.pi, 0.], [0., -np.pi / 2, 0.]])
# 第三组
'''global center trajectory and offset'''

'''uav group initialization'''
uavs = []
for i in range(g_v['uav_num']):
    obs_in = rd3(use_freq=True,
                 omega=np.array([3.5, 3.4, 10]),
                 dim=3,
                 thresh=np.array([0.5, 0.5, 0.5]),
                 dt=g_v['dt'])
    obs_in.set_init(all_zero=True)
    obs_out = rd3(use_freq=True,
                  omega=np.array([4, 4, 4]),  # 实际实验这个数一定要小 0.9, 0.9, 0.9，或者从小往大调
                  dim=3,
                  thresh=np.array([0.5, 0.5, 0.5]),
                  dt=g_v['dt'])
    obs_out.set_init(all_zero=True)
    uav = usv_consensus(uav_param=uav_par,
                        ctrl_att_param=att_ctrl_param,
                        ctrl_pos_param=pos_ctrl_param,
                        adjacency=g_v['g_A'][i],
                        in_degree=g_v['g_D'][i],
                        communication=g_v['g_B'][i],
                        obs_att=obs_in,
                        obs_pos=obs_out,
                        is_ideal=g_v['g_ideal'])
    uavs.append(uav)
'''uav group initialization'''


def cal_g_eta_dot_eta():
    _res = np.zeros((g_v['uav_num'], 3))
    _dot_res = np.zeros((g_v['uav_num'], 3))
    for p in range(g_v['uav_num']):
        _res[p][:] = uavs[p].uav.eta()
        _dot_res[p][:] = uavs[p].uav.dot_eta()
    return _res, _dot_res


if __name__ == '__main__':
    '''1. generate uncertainty for all UAVs at all timesteps'''
    consensus_un = consensus_uncertainty_N(is_ideal=g_v['g_ideal'], dt=g_v['dt'], tm=g_v['g_tm'], num_uav=g_v['uav_num'])  # (20000, 24)

    while g_v['g_t'] < g_v['g_tm'] - g_v['dt'] / 2:
        if g_v['g_N'] % int(1 / g_v['dt']) == 0:
            print('time: %.2f s.' % (g_v['g_N'] / int(1 / g_v['dt'])))

        '''2. calculations for each UAV'''
        g_eta, g_dot_eta = cal_g_eta_dot_eta()  # 先计算全局状态

        for i in range(g_v['uav_num']):  # 对于每一个无人机
            '''2.1 generate reference command, uncertainty, and bias for each uav'''
            dis_i = consensus_un[g_v['g_N'], 6 * i: 6 * (i + 1)]
            ref, dot_ref, _, _ = ref_uav(g_v['g_t'], ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)
            nu, dot_nu = offset_uavs(g_v['g_t'], offset_amplitude, offset_period, offset_bias_a, offset_bias_phase)  # 更新各个无人机的偏移量
            # nu, dot_nu = update_nu_dot_nu(g_v['g_t'])

            eta_d_i = ref[0: 3]  # eta 表示外环，d表示参考，i表示无人机编号
            dot_eta_d_i = dot_ref[0: 3]  # dot 表示一阶导数，eta表示外环，d表示参考，i表示无人机编号

            if not uavs[i].is_ideal:
                syst_dynamic_i = -uavs[i].uav.kt / uavs[i].uav.m * uavs[i].uav.dot_eta() + uavs[i].uav.A()
                obs_eta_i, _ = uavs[i].obs_pos.observe(x=uavs[i].uav.eta(), syst_dynamic=syst_dynamic_i)
            else:
                obs_eta_i = np.zeros(3)

            '''2.2 calculate consensus error'''
            uavs[i].ctrl_pos.control_update_outer_consensus(g_eta,
                                                            g_dot_eta,
                                                            nu,
                                                            dot_nu,
                                                            eta_d_i,
                                                            dot_eta_d_i,
                                                            uavs[i].uav.eta(),
                                                            uavs[i].uav.dot_eta(),
                                                            nu[i],
                                                            dot_nu[i],
                                                            uavs[i].d,
                                                            uavs[i].b,
                                                            uavs[i].adjacency,
                                                            uavs[i].uav.kt,
                                                            uavs[i].uav.m,
                                                            obs_eta_i)  # 无人机外环控制
            yyf_acc = uavs[i].ctrl_pos.control_out_consensus

            '''2.3 transfer virtual control command to actual throttle, phi_d, and theta_d'''
            phi_d_old = uavs[i].rho_d[0]
            theta_d_old = uavs[i].rho_d[1]
            phi_d, theta_d, throttle = uo_2_ref_angle_throttle(uo=uavs[i].ctrl_pos.control_out_consensus,
                                                               att=uavs[i].uav.uav_att(),
                                                               m=uavs[i].uav.m,
                                                               g=uavs[i].uav.g)

            dot_phi_d = (phi_d - phi_d_old) / uavs[i].uav.dt
            dot_theta_d = (theta_d - theta_d_old) / uavs[i].uav.dt
            uavs[i].rho_d = np.array([phi_d, theta_d, ref[3]])  # phi_d theta_d psi_d
            uavs[i].dot_rho_d = np.array([dot_phi_d, dot_theta_d, dot_ref[3]])  # phi_d theta_d psi_d 的一阶导数
            uavs[i].throttle = throttle

            '''2.4 inner loop control'''
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

            '''2.5 rk44 update'''
            action_4_uav_i = np.array([throttle, uavs[i].ctrl_att.control_in[0], uavs[i].ctrl_att.control_in[1], uavs[i].ctrl_att.control_in[2]])
            uavs[i].uav.rk44(action=action_4_uav_i, dis=dis_i, n=1, att_only=False)

            '''2.6 data record'''
            data_block_i = {'time': uavs[i].uav.time,
                            'control': action_4_uav_i,
                            'ref_angle': uavs[i].rho_d,
                            'ref_pos': ref[0: 3] + nu[i],
                            'ref_vel': dot_ref[0: 3] + dot_nu[i],
                            'd_in': np.array([0., 0., np.dot(uavs[i].uav.W(), np.array([dis_i[3], dis_i[4], dis_i[5]]))[2]]),
                            'd_in_obs': obs_rho_i,
                            'd_in_e_1st': uavs[i].obs_att.obs_error,
                            'd_out': np.array([dis_i[0], dis_i[1], dis_i[2]]) / uavs[i].uav.m,
                            'd_out_obs': obs_eta_i,
                            'd_out_e_1st': uavs[i].obs_pos.obs_error,
                            'state': uavs[i].uav.uav_state_call_back()}
            uavs[i].data_record.record(data=data_block_i)

        g_v['g_t'] += g_v['dt']  # time update
        g_v['g_N'] += 1  # global index update

    '''3. draw curve'''
    DRAW = True
    if DRAW:
        data_block = []
        for _uav in uavs:
            data_block.append(_uav.data_record)
        plot_consensus_pos(data_block)
        # plot_consensus_vel(data_block)
        plot_consensus_att(data_block)
        # plot_consensus_throttle(data_block)
        # plot_consensus_torque(data_block)
        plot_consensus_outer_obs(data_block)
    plt.show()

    SAVE = True
    if SAVE:
        os.mkdir(new_path)
        for i in range(g_v['uav_num']):
            path = new_path + '\\uav_' + str(i) + '/' if windows else new_path + '/uav_' + str(i) + '/'
            os.mkdir(path)
            uavs[i].data_record.package2file(path)
