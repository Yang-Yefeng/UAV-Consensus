import numpy as np
import pandas as pd
import sys, os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")


if __name__ == '__main__':
    A_num = 61
    T_num = 15
    ref_A = np.linspace(0, 60 * np.pi / 180, A_num)
    ref_T = np.linspace(3, 6, T_num)
    
