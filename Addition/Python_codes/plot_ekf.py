import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 8

sim_data_available = False

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY, use_custom_layout=False):
    global sim_data_available
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../experiment_data_check/"

    ## read files
    data_body_pos_des = \
    np.genfromtxt(file_path+'body_pos_des.txt', delimiter=None, dtype=(float))    
    data_body_vel_des = \
    np.genfromtxt(file_path+'body_vel_des.txt', delimiter=None, dtype=(float))
    data_body_ori_des = \
    np.genfromtxt(file_path+'body_ori_des.txt', delimiter=None, dtype=(float)) 

    data_sim_imu_pos = None
    data_sim_imu_vel = None    
    try:
        data_sim_imu_pos = \
        np.genfromtxt(file_path+'sim_imu_pos.txt', delimiter=None, dtype=(float))    
        data_sim_imu_vel = \
        np.genfromtxt(file_path+'sim_imu_vel.txt', delimiter=None, dtype=(float))
        sim_data_available = True
    except:
        print "\n Note: simulated data for position and/or velocity is not available. Will not plot simulated ground truth data\n"
        sim_data_available = False


    data_com = \
    np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))
    data_kinematics_com_vel = \
    np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))
 
    data_body_pos = \
    np.genfromtxt(file_path+'ekf_o_r.txt', delimiter=None, dtype=(float))
    data_body_vel = \
    np.genfromtxt(file_path+'ekf_o_v.txt', delimiter=None, dtype=(float))
    data_body_ori = \
    np.genfromtxt(file_path+'ekf_o_q_b.txt', delimiter=None, dtype=(float))

    data_accumulated_body_ori = \
    np.genfromtxt(file_path+'body_ori.txt', delimiter=None, dtype=(float))


    data_z_lfoot_pos = \
    np.genfromtxt(file_path+'ekf_z_l_foot_B.txt', delimiter=None, dtype=(float))
    data_z_rfoot_pos = \
    np.genfromtxt(file_path+'ekf_z_r_foot_B.txt', delimiter=None, dtype=(float))

    data_p_lfoot_pos = \
    np.genfromtxt(file_path+'ekf_p_left_B.txt', delimiter=None, dtype=(float))
    data_p_rfoot_pos = \
    np.genfromtxt(file_path+'ekf_p_right_B.txt', delimiter=None, dtype=(float))


    data_f_imu = \
    np.genfromtxt(file_path+'ekf_f_imu.txt', delimiter=None, dtype=(float))
    data_bias_f_imu = \
    np.genfromtxt(file_path+'ekf_B_bf.txt', delimiter=None, dtype=(float))


    data_omega_imu = \
    np.genfromtxt(file_path+'ekf_omega_imu.txt', delimiter=None, dtype=(float))    
    data_bias_omega_imu = \
    np.genfromtxt(file_path+'ekf_B_bw.txt', delimiter=None, dtype=(float))    

    data_meas_diff = \
    np.genfromtxt(file_path+'ekf_measured_diff.txt', delimiter=None, dtype=(float))    


    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

    st_idx = 10
    end_idx = len(data_x) - 1
    data_x = data_x[st_idx:end_idx]

    # PHASE MARKER #
    data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    # get phase.txt data #
    phseChange = []
    for i in range(0,len(data_x)-1):
            if data_phse[i] != data_phse[i+1]:
                phseChange.append(i - st_idx)
            else:
                pass
    axes = plt.gca()
 
    ## plot command/jpos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('body world pos(m)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        if sim_data_available:
            plt.plot(data_x, data_sim_imu_pos[st_idx:end_idx,i-1], "r-")

        plt.plot(data_x, data_body_pos[st_idx:end_idx,i-1], "b-", \
                 data_x, data_com[st_idx:end_idx, i-1], "c-")

        # plt.legend(('command', 'pos'), loc='upper left')
        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
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
    fig.canvas.set_window_title('body vel(m/s)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)

        if sim_data_available:
            plt.plot(data_x, data_sim_imu_vel[st_idx:end_idx,i-1], "r-")

        plt.plot(data_x, data_body_vel[st_idx:end_idx, i-1], "b-", \
                 data_x, data_kinematics_com_vel[st_idx:end_idx, i-1], "c-")


        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.xlabel('time (sec)')
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
                data_x, data_body_ori[st_idx:end_idx,i-1], "b-", \
                data_x, data_accumulated_body_ori[st_idx:end_idx, i-1], "c-")
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1


    # plt.get_current_fig_manager().window.wm_geometry("480x600+0+650")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body imu input and bias (acc)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_f_imu[st_idx:end_idx,i-1], "b-" , \
                data_x, data_bias_f_imu[st_idx:end_idx,i-1], "c-")
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    # Use another row
    if use_custom_layout:
        col_index = 0
        row_index = 1

    # plt.get_current_fig_manager().window.wm_geometry("480x600+0+650")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body omega input and bias (ang vel)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_omega_imu[st_idx:end_idx,i-1], "b-" , \
                data_x, data_bias_omega_imu[st_idx:end_idx,i-1], "c-")
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1


    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('measured diff (m)')
    for i in range(1,7,1):
        ax1 = plt.subplot(6, 1, i)
        plt.plot(data_x, 0*data_meas_diff[st_idx:end_idx,i-1], "r-")
        plt.plot(data_x, data_meas_diff[st_idx:end_idx,i-1], "b-")

        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1



    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('Body frame lfoot position (m)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_z_lfoot_pos[st_idx:end_idx,i-1], "r-")
        plt.plot(data_x, data_p_lfoot_pos[st_idx:end_idx,i-1], "b-")        
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('Body frame rfoot position (m)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_z_rfoot_pos[st_idx:end_idx,i-1], "r-")
        plt.plot(data_x, data_p_rfoot_pos[st_idx:end_idx,i-1], "b-")        
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1        



if __name__ == "__main__":
    create_figures(subfigure_height=500, use_custom_layout=True)
    plt.show()
