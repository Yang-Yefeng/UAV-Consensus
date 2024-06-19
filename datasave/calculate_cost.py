import sys, os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from utils.utils import *


if __name__ == '__main__':
    cur_path = os.path.dirname(os.path.abspath(__file__))
    base_dir = cur_path + '/pos_consensus_no_rl/'  # select the directory
    data_record = []
    pos_record = []
    uav_num = len(os.listdir(base_dir))     # check the number of the uav
    cost = 0.
    Q_pos = 1
    Q_vel = 0.01
    for i in range(uav_num):
        temp = data_collector(0)
        temp.load_file(base_dir + 'uav_%d/' % (i))

        pos_e = temp.state[:, 0: 3] - temp.ref_pos
        cost_pos_e = np.sum(pos_e ** 2) * Q_pos
        
        vel_e = temp.state[:, 3: 6] - temp.ref_vel
        cost_vel_e = np.sum(vel_e ** 2) * Q_vel
        cost += (cost_pos_e + cost_vel_e)
        print('uav %.0f: %.3f' % (i, cost_pos_e + cost_vel_e))
    print('total cost:', cost)
