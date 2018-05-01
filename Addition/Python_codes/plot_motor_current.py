
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 3

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index


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
    # fig = plt.figure(1)
    # plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('motor current (right_leg)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot( \
                data_x,  data_motor_current[st_idx:end_idx, i-1], "k-");
        # plt.legend(('command', 'pos'), loc='upper left')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    # fig = plt.figure(2)
    # plt.get_current_fig_manager().window.wm_geometry("480x600+400+0")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))       
    fig.canvas.set_window_title('motor_current (left_leg)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_motor_current[st_idx:end_idx, i-1 + 3], "k-")
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    current_sum = data_motor_current[st_idx:end_idx, 0];
    for i in range(1,6,1):
        current_sum = current_sum + abs(data_motor_current[st_idx:end_idx, i]);

    # fig = plt.figure(3)
    # plt.get_current_fig_manager().window.wm_geometry("480x300+800+0")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('sum of motor current')
    plt.plot(data_x, current_sum, "r-")
    plt.xlabel('time (sec)')

if __name__ == "__main__":
    create_figures()
    plt.show()


