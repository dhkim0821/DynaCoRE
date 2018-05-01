import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

file_path = os.getcwd() + "/../../experiment_data_check/"

## read files
data_cmd = \
np.genfromtxt(file_path+'command.txt', delimiter=None, dtype=(float))
data_torque = \
np.genfromtxt(file_path+'torque.txt', delimiter=None, dtype=(float))
data_motor_current = \
np.genfromtxt(file_path+'motor_current.txt', delimiter=None, dtype=(float))
speed_ratio_lin = 200;
speed_ratio_rot = 2942* 0.034;
torque_const = 0.039;
eff = -0.7
scale = [speed_ratio_lin * torque_const*eff, \
         speed_ratio_rot * torque_const*eff, \
         speed_ratio_rot * torque_const*eff];
data_qddot_cmd = \
np.genfromtxt(file_path+'qddot_cmd.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

st_idx = 10
end_idx = len(data_x) - 10
data_x = data_x[st_idx:end_idx]

## plot command/jpos
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('jtorque (right_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot( \
            data_x, scale[i-1] * data_motor_current[st_idx:end_idx, i-1], "k-", \
            data_x, data_cmd[st_idx:end_idx, i-1 + 6], "c-", \
            data_x, data_cmd[st_idx:end_idx,i-1], "r-", \
            data_x, data_torque[st_idx:end_idx,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+540+0")
fig.canvas.set_window_title('jtorque (left_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(\
            data_x, scale[i-1] * data_motor_current[st_idx:end_idx, i-1 + 3], "k-", \
            data_x, data_cmd[st_idx:end_idx, i-1+3 + 6], "c-", \
            data_x, data_cmd[st_idx:end_idx,i-1 + 3], "r-" , \
            data_x, data_torque[st_idx:end_idx,i-1 + 3], "b-")
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(3)
plt.get_current_fig_manager().window.wm_geometry("480x1226+1020+0")
fig.canvas.set_window_title('qddot (floating)')
for i in range(1,7,1):
    ax1 = plt.subplot(6, 1, i)
    plt.plot(data_x, data_qddot_cmd[st_idx:end_idx, i-1], "r-")
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(4)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+650")
fig.canvas.set_window_title('qddot (right)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_qddot_cmd[st_idx:end_idx, i-1 + 6], "r-")
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(5)
plt.get_current_fig_manager().window.wm_geometry("480x600+540+650")
fig.canvas.set_window_title('qddot (left)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_qddot_cmd[st_idx:end_idx, i-1 + 9], "r-")
    plt.grid(True)
plt.xlabel('time (sec)')


plt.show()

