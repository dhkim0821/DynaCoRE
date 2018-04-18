import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# file_path = '/Users/donghyunkim/Repository/dynacore/experiment_data_check/'
file_path = '/home/hcrl/Repository/dynacore/experiment_data_check/'
## read files
data_cmd = \
np.genfromtxt(file_path+'command.txt', delimiter=None, dtype=(float))
data_torque = \
np.genfromtxt(file_path+'torque.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

## plot command/jpos
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('jtorque (right_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_cmd[:,i-1], "r-", \
             data_x, data_torque[:,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+480+0")
fig.canvas.set_window_title('jtorque (left_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_cmd[:,i-1 + 3], "r-" , \
             data_x, data_torque[:,i-1 + 3], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

plt.show()

