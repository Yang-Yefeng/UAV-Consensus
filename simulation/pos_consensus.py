import os
import sys
import datetime
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.BS_FNTSMC import bs_fntsmc, bs_fntsmc_param
from uav.uav import UAV, uav_param
from utils.ref_cmd import *
from utils.utils import *
from utils.collector import data_collector

'''global variables'''
g_dt = 0.001        # global sampling period
g_t = 0.            # global time
g_tm = 20           # global maximum simulation time
g_ideal = True      # global ideal
g_obs_in = 'rd3'
g_obs_out = 'rd3'
'''global variables'''

'''Parameter list of the quadrotor'''
DT = g_dt
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
uav_param.time_max = g_tm
'''Parameter list of the quadrotor'''

'''Parameter list of the attitude controller'''
att_ctrl_param = bs_fntsmc_param(
    k1=np.array([5, 5, 5]),
    k2=np.array([8., 8., 20.]),
    k3=np.array([1., 1., 1.5]),
    k4=np.array([0.05, 0.05, 0.05]),
    k5=np.array([5, 5, 5]),  # 要大
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
    k1=np.array([6, 6, 4]),
    k2=np.array([0.3, 0.3, 1.0]),
    k3=np.array([0.5, 0.5, 1]),
    k4=np.array([0.05, 0.05, 0.05]),
    k5=np.array([8, 8, 8]),
    alpha1=np.array([1.2, 1.2, 1.5]),
    alpha2=np.array([1.2, 1.2, 1.2]),
    dim=3,
    dt=DT
)
'''Parameter list of the position controller'''

