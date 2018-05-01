
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

file_path = os.getcwd() + "/../../experiment_data_check/"

## read files
data_motor_current = \
np.genfromtxt(file_path+'motor_current.txt', delimiter=None, dtype=(float))
speed_ratio_lin = 200;
speed_ratio_rot = 2942* 0.034;
torque_const = 0.039;
eff = -0.7
scale = [speed_ratio_lin * torque_const*eff, \
         speed_ratio_rot * torque_const*eff, \
         speed_ratio_rot * torque_const*eff];
data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

st_idx = 10
end_idx = len(data_x) - 10
data_x = data_x[st_idx:end_idx]

## plot command/jpos
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('motor current (right_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot( \
            data_x,  data_motor_current[st_idx:end_idx, i-1], "k-");
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+400+0")
fig.canvas.set_window_title('motor_current (left_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_motor_current[st_idx:end_idx, i-1 + 3], "k-")
    plt.grid(True)
plt.xlabel('time (sec)')


current_sum = data_motor_current[st_idx:end_idx, 0];
for i in range(1,6,1):
    current_sum = current_sum + abs(data_motor_current[st_idx:end_idx, i]);


fig = plt.figure(3)
plt.get_current_fig_manager().window.wm_geometry("480x300+800+0")
fig.canvas.set_window_title('sum of motor current')
plt.plot(data_x, current_sum, "r-")
plt.xlabel('time (sec)')
plt.show()
