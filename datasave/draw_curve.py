import sys, os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import matplotlib.pyplot as plt
from utils.collector import data_collector


if __name__ == '__main__':
    uav_num = 4
    cur_path = os.path.dirname(os.path.abspath(__file__))
    dir_name = cur_path + '/pos_consensus-2024-05-31-13-21-02/'  # select the directory
    data_record = []
    for i in range(uav_num):
        temp = data_collector(0)
        temp.load_file(dir_name + 'uav_%d/' % (i))
        data_record.append(temp)

    index = 3
    data_record[index].plot_pos()
    data_record[index].plot_att()
    # data_record[index].plot_torque()
    # data_record[index].plot_throttle()
    # data_record[0].plot_inner_obs()

    plt.show()
