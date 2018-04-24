import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# file_path = '/Users/donghyunkim/Repository/dynacore/experiment_data_check/'
file_path = '/home/hcrl/Repository/dynacore/experiment_data_check/'
## read files
data_foot_pos_des = \
np.genfromtxt(file_path+'rfoot_pos_des.txt', delimiter=None, dtype=(float))
data_foot_pos = \
np.genfromtxt(file_path+'rfoot_pos.txt', delimiter=None, dtype=(float))

data_foot_vel_des = \
np.genfromtxt(file_path+'rfoot_vel_des.txt', delimiter=None, dtype=(float))
data_foot_vel = \
np.genfromtxt(file_path+'rfoot_vel.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

st_idx = 300
end_idx = len(data_x) - 500
data_x = data_x[st_idx:end_idx]

## plot foot pos
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('foot pos')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_foot_pos_des[st_idx:end_idx,i-1], "r-", \
             data_x, data_foot_pos[st_idx:end_idx,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

## plot foot vel
fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+480+0")
fig.canvas.set_window_title('foot vel')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_foot_vel_des[st_idx:end_idx,i-1], "r-", \
             data_x, data_foot_vel[st_idx:end_idx,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')


plt.show()

