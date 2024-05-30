import matplotlib.pyplot as plt
from utils.collector import data_collector


if __name__ == '__main__':
    uav_num = 1
    dir_name = './pos_consensus-2024-05-30-16-41-19/'  # select the directory
    data_record = []
    for i in range(uav_num):
        temp = data_collector(0)
        temp.load_file(dir_name + 'uav_%d/' % (i))
        data_record.append(temp)

    data_record[0].plot_pos()
    data_record[0].plot_att()
    data_record[0].plot_torque()
    data_record[0].plot_throttle()
    data_record[0].plot_torque()
    # data_record[0].plot_inner_obs()

    plt.show()
