import numpy as np
from utils.utils import *
import sys, os
from typing import Union

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from observer.RobustDifferentatior_3rd import robust_differentiator_3rd as rd3
from controller.BS_FNTSMC import bs_fntsmc, bs_fntsmc_param
from uav.uav import UAV, uav_param
from utils.ref_cmd import *
from utils.utils import *
from utils.collector import data_collector


class usv_consensus:
    def __init__(self,
                 uav_param: uav_param,                  # parameters of a uav
                 ctrl_att_param: bs_fntsmc_param,       # parameters of attitude controller
                 ctrl_pos_param: bs_fntsmc_param,       # parameters of position controller
                 adjacency: Union[np.ndarray, list],    # adjacency of this uav
                 in_degree: float,                      # in-degree of this uav
                 communication: float,                  # communication of this uav
                 obs_att: rd3,                          # observer of attitude loop
                 obs_pos: rd3,                          # observer of position loop
                 is_ideal: bool                         # exist disturbance of not
                 ):
        self.uav = UAV(uav_param)
        self.ctrl_att = bs_fntsmc(ctrl_att_param)
        self.ctrl_pos = bs_fntsmc(ctrl_pos_param)
        self.obs_att = obs_att
        self.obs_pos = obs_pos
        self.adjacency = adjacency
        self.d = in_degree
        self.b = communication
        self.is_ideal = is_ideal
        self.data_record = data_collector(N=int(self.uav.time_max / self.uav.dt))
        