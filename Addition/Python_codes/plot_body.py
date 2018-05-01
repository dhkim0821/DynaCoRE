import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 4

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

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
    data_qdot = \
    np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))

    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

    st_idx = 250
    end_idx = len(data_x) - 10
    data_x = data_x[st_idx:end_idx]

    ## plot command/jpos
    # fig = plt.figure(1)
    # plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")

    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('com pos')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_com_des[st_idx:end_idx,i-1], "r-", \
                data_x, data_com[st_idx:end_idx,i-1], "b-")
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
    # plt.get_current_fig_manager().window.wm_geometry("480x600+540+0")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('com vel')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_com_vel_des[st_idx:end_idx,i-1], "r-" , \
                data_x, data_com_vel[st_idx:end_idx,i-1], "b-")
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
        
    # fig = plt.figure(3)
    # plt.get_current_fig_manager().window.wm_geometry("480x600+0+650")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body ori (quaternion)')
    for i in range(1,5,1):
        ax1 = plt.subplot(4, 1, i)
        plt.plot(data_x, data_body_ori_des[st_idx:end_idx,i-1], "r-" , \
                data_x, data_body_ori[st_idx:end_idx,i-1], "b-")
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    # fig = plt.figure(4)
    # plt.get_current_fig_manager().window.wm_geometry("480x600+540+650")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body ang vel')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_qdot[st_idx:end_idx,i-1+3], "r-")
        plt.grid(True)
    plt.xlabel('time (sec)')


if __name__ == "__main__":
    create_figures()
    plt.show()