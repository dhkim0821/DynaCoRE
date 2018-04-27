import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

file_path = os.getcwd() + "/../../experiment_data_check/"

## read files
data_global_pos_offset = \
np.genfromtxt(file_path+'global_pos_local.txt', delimiter=None, dtype=(float))
data_estimated_com = \
np.genfromtxt(file_path+'estimated_com_state.txt', delimiter=None, dtype=(float))
data_com_des = \
np.genfromtxt(file_path+'com_pos_des.txt', delimiter=None, dtype=(float))
data_com = \
np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))
data_body_ori_des = \
np.genfromtxt(file_path+'body_ori_des.txt', delimiter=None, dtype=(float))
data_body_ori = \
np.genfromtxt(file_path+'body_ori.txt', delimiter=None, dtype=(float))
data_com_vel_des = \
np.genfromtxt(file_path+'com_vel_des.txt', delimiter=None, dtype=(float))
data_com_vel = \
np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

st_idx = 30
end_idx = len(data_x) - 10
data_x = data_x[st_idx:end_idx]

## plot command/jpos
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('com pos')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_com_des[st_idx:end_idx,i-1], "r-", \
            data_x, data_com[st_idx:end_idx,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+400+0")
fig.canvas.set_window_title('com vel')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_com_vel_des[st_idx:end_idx,i-1], "r-" , \
            data_x, data_com_vel[st_idx:end_idx,i-1], "b-")
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(3)
plt.get_current_fig_manager().window.wm_geometry("480x600+800+0")
fig.canvas.set_window_title('body ori (quaternion)')
for i in range(1,5,1):
    ax1 = plt.subplot(4, 1, i)
    plt.plot(data_x, data_body_ori_des[st_idx:end_idx,i-1], "r-" , \
            data_x, data_body_ori[st_idx:end_idx,i-1], "b-")
    plt.grid(True)
plt.xlabel('time (sec)')


plt.show()

