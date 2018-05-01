import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# number of figures in this plot
num_figures = 4

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_row_index=0):
    figure_number = starting_figure_no
    figure_index = 0
    row_index = starting_row_index

    ## read files --------------------------------------------------------------------
    file_path = os.getcwd() + "/../../experiment_data_check/"

    data_jpos_des = \
    np.genfromtxt(file_path+'jpos_des.txt', delimiter=None, dtype=(float))
    data_config = \
    np.genfromtxt(file_path+'config.txt', delimiter=None, dtype=(float))
    data_jvel_des = \
    np.genfromtxt(file_path+'jvel_des.txt', delimiter=None, dtype=(float))
    data_qdot = \
    np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))
    data_jjvel = \
    np.genfromtxt(file_path+'joint_jvel.txt', delimiter=None, dtype=(float))
    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))
    ##--------------------------------------------------------------------------------


    # Plot Figure --------------------------------------------------------------------
    ## plot command/jpos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*figure_index) + "+" + str(subfigure_height*row_index))
    #plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
    fig.canvas.set_window_title('jpos (right_leg)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_jpos_des[:,i-1], "r-", \
                 data_x, data_config[:,i-1 + 6], "b-")
        # plt.legend(('command', 'pos'), loc='upper left')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    figure_index += 1
    #----------------------------------------------------------------------------------

    # Plot Figure --------------------------------------------------------------------
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*figure_index) + "+" + str(subfigure_height*row_index))    
    #plt.get_current_fig_manager().window.wm_geometry("480x600+480+0")
    fig.canvas.set_window_title('jpos (left_leg)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_jpos_des[:,i-1 + 3], "r-" , \
                 data_x, data_config[:,i-1 + 9], "b-")
        # plt.legend(('command', 'pos'), loc='upper left')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    figure_index += 1
    #----------------------------------------------------------------------------------

    # Plot Figure --------------------------------------------------------------------
    ## plot jvel
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*figure_index) + "+" + str(subfigure_height*row_index))    
    #plt.get_current_fig_manager().window.wm_geometry("480x600+960+0")
    fig.canvas.set_window_title('jvel (right_leg)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_jjvel[:,i-1], "c-", \
                 data_x, data_qdot[:,i-1 + 6], "b-", \
                 data_x, data_jvel_des[:, i-1], "r-")
        # plt.legend(('command', 'pos'), loc='upper left')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    figure_index += 1
    #----------------------------------------------------------------------------------

    # Plot Figure --------------------------------------------------------------------
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*figure_index) + "+" + str(subfigure_height*row_index))        
    #plt.get_current_fig_manager().window.wm_geometry("480x600+1440+0")
    fig.canvas.set_window_title('jvel (left_leg)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_jjvel[:,i-1 + 3], "c-" , \
                 data_x, data_qdot[:,i-1 + 9], "b-", \
                 data_x, data_jvel_des[:, i-1 + 3], "r-");
        # plt.legend(('command', 'pos'), loc='upper left')
        plt.grid(True)
    plt.xlabel('time (sec)')


if __name__ == "__main__":
    create_figures()
    plt.show()

