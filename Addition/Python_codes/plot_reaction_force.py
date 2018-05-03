import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os


file_path = os.getcwd() + "/../../experiment_data_check/"
## read files
data_rf_cmd = \
np.genfromtxt(file_path+'reaction_force.txt', delimiter=None, dtype=(float))
data_rf_sense = \
np.genfromtxt(file_path+'reaction_force.txt', delimiter=None, dtype=(float))

data_phase = np.genfromtxt(file_path+'phase.txt', delimiter='\n', dtype=(float))
data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))




st_idx = 10;
end_idx = len(data_x) - 100;
data_x = data_x[st_idx:end_idx];

## plot command/jpos
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('reaction_force (right_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_rf_cmd[st_idx:end_idx,i-1], "r-", \
             data_x, data_rf_sense[st_idx:end_idx,i-1], "b-", \
             data_x, data_phase[st_idx:end_idx], "k-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+480+0")
fig.canvas.set_window_title('reaction_force (left_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_rf_cmd[st_idx:end_idx,i-1 + 3], "r-" , \
             data_x, data_rf_sense[st_idx:end_idx,i-1 + 3], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

plt.show()

