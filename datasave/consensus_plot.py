import sys, os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import mpl_toolkits.mplot3d.axes3d as axes3d
from matplotlib.pyplot import MultipleLocator
from utils.utils import *


if __name__ == '__main__':
    cur_path = os.path.dirname(os.path.abspath(__file__))
    base_dir = cur_path + '/pos_consensus_rl/'  # select the directory
    data_record = []
    pos_record = []
    uav_num = len(os.listdir(base_dir))     # check the number of the uav
    for i in range(uav_num):
        temp = data_collector(0)
        temp.load_file(base_dir + 'uav_%d/' % (i))
        data_record.append(temp)
        s = temp.state[:, 0: 3]    # get position
        pos_record.append(s)

    pos_record = np.array(pos_record)
    xbound = np.array([int(np.min(pos_record[:, :, 0])) - 1, int(np.max(pos_record[:, :, 0])) + 1])
    ybound = np.array([int(np.min(pos_record[:, :, 1])) - 1, int(np.max(pos_record[:, :, 1])) + 1])
    zbound = np.array([int(np.min(pos_record[:, :, 2])) - 1, int(np.max(pos_record[:, :, 2])) + 1])

    fig = plt.figure(figsize=(9, 9))
    ax = axes3d.Axes3D(fig)
    ax.set_aspect('auto')

    ax.set_xlim3d(xbound)
    ax.set_ylim3d(ybound)
    ax.set_zlim3d(zbound)
    ax.xaxis.set_major_locator(MultipleLocator(2))
    ax.yaxis.set_major_locator(MultipleLocator(2))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Quadrotor Consensus Simulation', fontsize='13')

    color = ['red', 'green', 'blue', 'orange']

    for i in range(uav_num):
        traj = ax.plot([], [], [], color=color[i], linewidth=1.5)[0]
        traj.set_data(pos_record[i, :, 0], pos_record[i, :, 1])
        traj.set_3d_properties(pos_record[i, :, 2])

    plot_consensus_pos(data_record)
    plot_consensus_att(data_record)
    plot_consensus_outer_obs(data_record)

    plt.show()
    # plt.pause(0.000000001)