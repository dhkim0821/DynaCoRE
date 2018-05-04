import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 2

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index


    file_path = os.getcwd() + "/../../experiment_data_check/"

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
    # PHASE MARKER #
    data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    # get phase.txt data #
    phseChange = []
    for i in range(0,len(data_phse)-1):
            if data_phse[i] != data_phse[i+1]:
                phseChange.append(i+1)
            else:
                pass
    axes = plt.gca()

    st_idx = 1
    end_idx = len(data_x) - 10
    data_x = data_x[st_idx:end_idx]

    ## plot foot pos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('foot pos')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_foot_pos_des[st_idx:end_idx,i-1], "r-", \
                 data_x, data_foot_pos[st_idx:end_idx,i-1], "b-")
        # plt.legend(('command', 'pos'), loc='upper left')
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1    

    ## plot foot vel
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('foot vel')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_foot_vel_des[st_idx:end_idx,i-1], "r-", \
                 data_x, data_foot_vel[st_idx:end_idx,i-1], "b-")
        # plt.legend(('command', 'pos'), loc='upper left')
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1    


if __name__ == "__main__":
    create_figures()
    plt.show()
