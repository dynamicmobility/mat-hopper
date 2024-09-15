from copy import deepcopy
import numpy as np
import pandas as pd
import math
import seaborn as sns
from pypalettes import get_hex
import matplotlib.pyplot as plt
from single_leg_hopper_code.history import *
from subprocess import call
import os, re
from scipy import stats

def make_plots(df, behavior, condition_names, labels, palette):
    
    # Get all axis limits
    # power_limits, jerk_limits, tracking_limits, height_limits = get_axis_limits(df)
    
    # # Make the plots
    fig = plt.figure()
    fig.tight_layout()  
    
    # # Segment out the data based on behavior
    # for condition in condition_names:
    #     df_condition = df[df['condition'] == condition]
    #     df_behavior = df_condition[df_condition['behavior'] == behavior]
    # # df_behavior = df[df['behavior'] == behavior] #DEBUGGING
    
    # plt.scatter(df_behavior["phase"], df_behavior["power"])
    # axs[row, col].legend(labels)
    # axs[row, col].set_xlabel('Hop Cycle')
    # axs[row, col].set_ylabel('Power (W)')
    # axs[row, col].set_ylim(power_limits)
    
    # Loop through each condition and plot the time series
    for condition in condition_names:
        df_condition = df[(df['behavior'] == behavior) & (df['condition'] == condition)]
        # print(df_condition)
        # if not df_condition.empty:
        #     print("phase: ", df_condition['phase'].values[0])
        #     plt.plot(df_condition['phase'].values[0], df_condition['power'].values[0], label=condition)
        plt.plot(df_condition['time'].values[0], df_condition['error'].values[0], label=labels[condition_names.index(condition)], color=palette[condition_names.index(condition)])
        
    # Shrink current axis by 20%
    ax = plt.gca()
    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    # Put a legend to the right of the current axis
    ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

    plt.xlabel("Phase (Normalized Time)")
    plt.ylabel("Tracking Error [m] ")
    plt.grid(True)
    
    return fig

# Assuming data1, data2, data3, data4, and data5 are your datasets
def get_exp_data(file_path, rows, behavior, condition_name):
    h = History()
    h.load_saved_dict(file_path)
            
    # Get average power consumption 
    u_hy = h.data.get('hipy_torque')
    angv_hy = h.data.get('hipy_dot')
    u_hx = h.data.get('hipy_torque')
    angv_hx = h.data.get('hipx_dot')
    power = np.sqrt(np.multiply(u_hy, angv_hy)**2 + np.multiply(u_hx, angv_hx)**2)
    
    # Get average jerk
    t = h.data.get('t')
    base_acc_z = h.data.get('base_acc_z')
    base_jerk_z = np.gradient(base_acc_z, t)
    
    # Get average tracking error
    base_x = np.array(h.data.get('x_base'))
    base_y = np.array(h.data.get('y_base'))
    base_x_des = np.array(h.data.get('x_base_des'))
    base_y_des = np.array(h.data.get('y_base_des'))
    tracking_error = np.sqrt((base_x - base_x_des)**2 + (base_y - base_y_des)**2)
    
    # Get average hop height
    base_z = h.data.get('z_base')

    # TO DO: LOOP THROUGH EACH STEP
    power_hop, jerk_hop, error_hop, height_hop, time_hop, normalized_time = [], [], [], [], [], []
    state = h.data.get('state')
    t0 = 0
    tf = 60
    indices = [i for i in range(1, len(state)) if state[i] == 1 and state[i-1] == 2 and t[i] >= t0 and t[i] <= tf]
                    
    rows.append({'behavior': behavior,
                    'condition': condition_name,
                    'power': power[indices[0]:indices[-1]],
                    'jerk': base_jerk_z[indices[0]:indices[-1]],
                    'error': tracking_error[indices[0]:indices[-1]],
                    'height': base_z[indices[0]:indices[-1]],
                    'phase': t[indices[0]:indices[-1]],
                    'time': t[indices[0]:indices[-1]]})
    
    return rows
    
def get_color_palette(general_folder, labels):
    
    grey_white = "#E6E6E6"
    dark_blue = "#5E61B3"
    light_blue = "#7779B4"
    lighter_blue = "#9899B9"
    dark_yellow = "#C6A667" 
    light_yellow = "#CFBE9B"
    light_green = "#848E73"
    dark_green = "#718055"
        
    palette = []
    other_counter = 0
    for label in labels:
        if "MD" in label:
            palette.append(grey_white)
        elif "PVC" in label:
            palette.append(dark_yellow)
        elif "AL" in label:
            palette.append(lighter_blue)
        elif "Ti" in label:
            palette.append(light_blue)
        elif "SS" in label:
            palette.append(dark_blue)
        elif "inc" in label or "increase" in label or other_counter == 0:
            palette.append(light_green)
            other_counter += 1
        elif "dec" in label or "decrease" in label or other_counter == 1:
            palette.append(dark_green)
           
    if palette == []:
        palette = get_hex("Acadia", keep_first_n=len(labels))
           
    return palette
    
def get_axis_limits(df):

    # Get all axis limits
    power_range = [df['power'].quantile(0.01), df['power'].quantile(1)]
    buffer = 0.1 * (power_range[1] - power_range[0]) + 1e-6
    power_limits = [power_range[0] - buffer, power_range[1] + buffer]
    
    jerk_range = [df['jerk'].quantile(0.01), df['jerk'].quantile(0.98)]
    buffer = 0.1 * (jerk_range[1] - jerk_range[0]) + 1e-6
    jerk_limits = [jerk_range[0] - buffer, jerk_range[1] + buffer]
    
    tracking_range = [df['error'].quantile(0.01), df['error'].quantile(1)]
    buffer = 0.1 * (tracking_range[1] - tracking_range[0]) + 1e-6
    tracking_limits = [tracking_range[0] - buffer, tracking_range[1] + buffer]
    
    height_range = [df['height'].quantile(0.01), df['height'].quantile(1)]
    buffer = 0.1 * (height_range[1] - height_range[0]) + 1e-6
    height_limits = [height_range[0] - buffer, height_range[1] + buffer]
    return power_limits, jerk_limits, tracking_limits, height_limits

def make_timeseries_plots(materials:list, exp3_characteristic):
            
    # Create a dataframe 
    df = pd.DataFrame(columns=['behavior', 'condition', 'power', 'power_std', 'jerk', 'jerk_std', 'error', 'error_std', 'height', 'height_std','phase','time'])
    rows = []
    
    # Compare MD, with each material separately, with functional gradient in time-series form
    
    # Loop through all experimental data and process it 
    general_folder = os.getcwd()
    exp1_folder = "Experiment_1"
    exp3_folder = "Experiment_3"
    
    for behavior in range(1, 5):
        pkl_name = "data_" + str(behavior) + ".pkl"
        mat = ["MD"]
        mat.extend(materials)
        exp1_results_folders = [os.path.join(general_folder, exp1_folder, material) for material in mat]
        labels = deepcopy(mat)
        flattened_folders = [os.path.join(b, item)for b in [os.path.join(general_folder, exp3_folder, f"{exp3_characteristic}_{a}")for a in ["Increase", "Decrease"]]for item in os.listdir(b)]
        labels.extend([f"{'ρ' if exp3_characteristic == "Density" else ('E' if exp3_characteristic == "Modulus" else "Mat")}-{a}" for a in ["inc", "dec"]])
        exp3_results_folders = [folder for folder in flattened_folders if f'{materials[0]}' in folder and f'{materials[1]}' in folder and f'{materials[2]}' in folder]
        condition_names = exp1_results_folders
        condition_names.extend(exp3_results_folders)
        
        for condition_name in condition_names: 
            # Process data 
            file_path = os.path.join(condition_name, pkl_name)
            rows = get_exp_data(file_path, rows, behavior, condition_name)

    # Store all experiment data in a dataframe
    df = pd.DataFrame(rows)
       
     # Custom color palette
    palette = get_color_palette(exp3_folder, labels)
    sns.set_palette(palette, n_colors=None, desat=None, color_codes=False)
    sns.set_style("white")
    sns.set_theme(style="whitegrid")
    plt.rcParams['figure.figsize'] = [8, 2]
    
    for behavior in range(1, 5):
        
        # Get colors based on conditions
        # for i in range(len(condition_names)):
        #     if "Increase" in condition_names[i]:
        #         condition_names[i] = "Increase"
        #     elif "Decrease" in condition_names[i]:
        #         condition_names[i] = "Decrease"
        #     else:
        #         condition_names[i] = "Material"
        
        fig = make_plots(df, behavior, condition_names, labels, palette)
        
        
        # Set title
        title = f"{exp3_characteristic} Functional Gradient: {materials[0]}, {materials[1]}, {materials[2]}"
        title_units = title 
        # fig1.suptitle(title_units)
        # fig2.suptitle(title_units)
        
        # Save figure
        folder_path = os.path.join(general_folder, "Experiment_3", "TimeSeriesPlots")
        os.makedirs(folder_path, exist_ok=True)
        plt.savefig(os.path.join(folder_path, f"comparison_{title}_behavior{behavior}.png"))
        plt.savefig(os.path.join(folder_path, f"comparison_{title}_behavior{behavior}.eps"), format='eps')