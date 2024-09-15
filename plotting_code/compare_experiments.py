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

# Assuming data1, data2, data3, data4, and data5 are your datasets

def run_wilkes_test(data):
    
    if len(data) > 2:
        # Perform Shapiro-Wilk test
        shapiro_results = stats.shapiro(data)

        # Print the test statistic and p-value
        print("Shapiro-Wilk Test Results:")
        print("Test Statistic:", shapiro_results.statistic)
        print("p-value:", shapiro_results.pvalue)

        # Check if the data passed the test
        if shapiro_results.pvalue > 0.05:
            print("The data passed the Shapiro-Wilk test.")
        else:
            print("The data did not pass the Shapiro-Wilk test.")
    else:
        print("not enough data")
            

def run_mannwhitney_test(data1, data2):
    
    if len(data1) > 2 and len(data2) > 0:
    # Perform Shapiro-Wilk test
        shapiro_results = stats.mannwhitneyu(data1, data2)

        # Print the test statistic and p-value
        # print("Mann-Whitney Test Results:")
        # print("Test Statistic:", shapiro_results.statistic)
        # print("p-value:", shapiro_results.pvalue)

        # Check if the data passed the test
        if shapiro_results.pvalue < 0.05:
            print("The data passed the Mann-Whitney U Test.")
        else:
            print("The data did not the Mann-Whitney U Test.")
        return shapiro_results.pvalue
    else:
        print("DATA1: ", len(data1))
        print("DATA2: ", len(data2))
        print("NOT ENOUGH DATA")
        return 1  

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
    t = np.subtract(t,t[0])
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
    power_hop, jerk_hop, error_hop, height_hop = [], [], [], []
    state = h.data.get('state')
    t0 = 20
    tf = 60
    print(t)
    indices = [i for i in range(1, len(state)) if state[i] == 1 and state[i-1] == 2 and t[i] >= t0 and t[i] <= tf]
    if indices == [] or len(indices) == 1:
        print("SIMULATION FAILED !!!!!!!!!!!!!!!!!!!!!!!!! ")
        power_hop = np.mean(power)
        jerk_hop = np.mean(abs(base_jerk_z))
        error_hop = np.mean(tracking_error)
        height_hop = 0
        
        rows.append({'behavior': behavior,
                        'condition': condition_name,
                        'power': power_hop,
                        'jerk': jerk_hop,
                        'error': error_hop,
                        'height': height_hop})
    else: 
        for i in range(len(indices)-1):
            power_hop = np.mean(power[indices[i]:indices[i+1]])
            jerk_hop = np.mean(abs(base_jerk_z[indices[i]:indices[i+1]]))
            error_hop = np.mean(tracking_error[indices[i]:indices[i+1]])
            height_hop = (max(base_z[indices[i]:indices[i+1]]))
            
            rows.append({'behavior': behavior,
                            'condition': condition_name,
                            'power': power_hop,
                            'jerk': jerk_hop,
                            'error': error_hop,
                            'height': height_hop})
    
    return rows

def make_whisker_plot(df, behavior, condition_names, labels, palette):     
    # Get all axis limits
    power_limits, jerk_limits, tracking_limits, height_limits = get_axis_limits(df)
    
    # Segment out the data based on behavior
    df_behavior = df[df['behavior'] == behavior]
    
    # Make the plots
    fig, axs = plt.subplots(2, 2)
    fig.tight_layout()  
        
    row = 0
    col = 0
    sns.boxplot(x=df_behavior["condition"], y=df_behavior["power"], ax=axs[row, col],hue =df_behavior["condition"], palette=palette,showfliers=False)
    for condition in condition_names:
        print(condition)
        # run_wilkes_test(df_behavior["power"][df_behavior["condition"] == condition])
        p = run_mannwhitney_test(df_behavior["power"][df_behavior["condition"] == condition_names[0]], df_behavior["power"][df_behavior["condition"] == condition])
        if p < 0.05:
            axs[row, col].text(condition_names.index(condition), df_behavior["power"][df_behavior["condition"] == condition].max(), "*", ha='center', va='bottom', fontsize=12)
    axs[row, col].set_xticks(range(len(condition_names)))
    axs[row, col].set_xticklabels(labels)
    axs[row, col].set_xlabel('Material')
    axs[row, col].set_ylabel('Power (W)')
    axs[row, col].set_ylim(power_limits)
    
    row = 0
    col = 1
    sns.boxplot(x=df_behavior["condition"], y=df_behavior["jerk"], ax=axs[row, col],hue =df_behavior["condition"], palette=palette,showfliers=False)
    for condition in condition_names:
        # run_wilkes_test(df_behavior["jerk"][df_behavior["condition"] == condition])
        p = run_mannwhitney_test(df_behavior["jerk"][df_behavior["condition"] == condition_names[0]], df_behavior["jerk"][df_behavior["condition"] == condition])
        if p < 0.05:
            axs[row, col].text(condition_names.index(condition), df_behavior["jerk"][df_behavior["condition"] == condition].max(), "*", ha='center', va='bottom', fontsize=12)
    axs[row, col].set_xticks(range(len(condition_names)))
    axs[row, col].set_xticklabels(labels)
    axs[row, col].set_xlabel('Material')
    axs[row, col].set_ylabel('Jerk $(m/s^3)$')
    axs[row, col].set_ylim(jerk_limits)
    
    row = 1
    col = 0
    sns.boxplot(x=df_behavior["condition"], y=df_behavior["error"], ax=axs[row, col],hue =df_behavior["condition"], palette=palette,showfliers=False)
    for condition in condition_names:
        # run_wilkes_test(df_behavior["error"][df_behavior["condition"] == condition])
        p = run_mannwhitney_test(df_behavior["error"][df_behavior["condition"] == condition_names[0]], df_behavior["error"][df_behavior["condition"] == condition])
        if p < 0.05:
            axs[row, col].text(condition_names.index(condition), df_behavior["error"][df_behavior["condition"] == condition].max(), "*", ha='center', va='bottom', fontsize=12)
    axs[row, col].set_xticks(range(len(condition_names)))
    axs[row, col].set_xticklabels(labels)
    axs[row, col].set_xlabel('Material')
    axs[row, col].set_ylabel('Tracking Error')
    axs[row, col].set_ylim(tracking_limits)

    row = 1
    col = 1
    sns.boxplot(x=df_behavior["condition"], y=df_behavior["height"], ax=axs[row, col],hue =df_behavior["condition"], palette=palette,showfliers=False)
    for condition in condition_names:
        # run_wilkes_test(df_behavior["height"][df_behavior["condition"] == condition])
        p = run_mannwhitney_test(df_behavior["height"][df_behavior["condition"] == condition_names[0]], df_behavior["height"][df_behavior["condition"] == condition])
        if p < 0.05:
            axs[row, col].text(condition_names.index(condition), df_behavior["height"][df_behavior["condition"] == condition].max(), "*", ha='center', va='bottom', fontsize=12)
    axs[row, col].set_xticks(range(len(condition_names)))
    axs[row, col].set_xticklabels(labels)
    axs[row, col].set_xlabel('Material')
    axs[row, col].set_ylabel('Hop Height (m)')
    axs[row, col].set_ylim(height_limits)    
    
    # Run statistical tests
    for condition in condition_names:
        # run_wilkes_test(df_behavior["height"][df_behavior["condition"] == condition])
        p = run_mannwhitney_test(df_behavior["height"][df_behavior["condition"] == condition_names[0]], df_behavior["height"][df_behavior["condition"] == condition])

    
    return fig, axs
        
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

def compare_experiments(general_folder, condition_names, labels):
        
    # Create a dataframe 
    df = pd.DataFrame(columns=['behavior', 'condition', 'power', 'power_std', 'jerk', 'jerk_std', 'error', 'error_std', 'height', 'height_std'])
    rows = []
    
    # Loop through all experimental data and process it 
    for behavior in range(1, 5):
        pkl_name = "data_" + str(behavior) + ".pkl"
        
        for condition_name in condition_names:
            
            # Process data 
            file_path = os.path.join(general_folder,condition_name,pkl_name)
            rows = get_exp_data(file_path, rows, behavior, condition_name)
    
    # Store all experiment data in a dataframe
    df = pd.DataFrame(rows)
    
    
    # Custom color palette
    palette = get_color_palette(general_folder, condition_names)
    sns.set_palette(palette, n_colors=None, desat=None, color_codes=False)
    sns.set_style("white")
    sns.set_theme(style="whitegrid")
    plt.rcParams['figure.figsize'] = [8, 4]
        
    for behavior in range(1, 5):
        
        # PLOT RESULTS
        fig, axs = make_whisker_plot(df, behavior, condition_names, labels, palette)
        plt.savefig(os.path.join(general_folder, "comparison_behavior" + str(behavior) + ".png"))
        plt.savefig(os.path.join(general_folder, "comparison_behavior" + str(behavior) + ".eps"), format='eps')

def compare_experiments_exp2(general_folder, condition_names, labels, density=True):
        
    # Create a dataframe 
    df = pd.DataFrame(columns=['behavior', 'condition', 'power', 'power_std', 'jerk', 'jerk_std', 'error', 'error_std', 'height', 'height_std'])
    rows = []
    
    # Loop through all experimental data and process it 
    for behavior in range(1, 5):
        pkl_name = "data_" + str(behavior) + ".pkl"
        
        for condition_name in condition_names:
            # Process data 
            file_path = os.path.join(general_folder,condition_name,pkl_name)
            rows = get_exp_data(file_path, rows, behavior, condition_name)
            
    # Store all experiment data in a dataframe
    df = pd.DataFrame(rows)     
        
    # Custom color palette
    # palette = get_color_palette(general_folder, labels)
    palette = get_hex("Acadia", keep_first_n=len(labels))
    sns.set_palette(palette, n_colors=None, desat=None, color_codes=False)
    sns.set_style("white")
    sns.set_theme(style="whitegrid")
    plt.rcParams['figure.figsize'] = [8, 4]
    
    for behavior in range(1, 5):
        
        # PLOT RESULTS
        fig, axs = make_whisker_plot(df, behavior, condition_names, labels, palette)
        
        # Modify the plots
        if density:
            xlabel = r"Youngs Modulus, E (GPa)"
            pattern = r"ρ=(\d+\.?\d*)"
            title = f'ρ = {[float(re.search(pattern, condition).group(1)) for condition in condition_names][0]}'
            title_units = title + r'$kg/m^3$'
        else:
            xlabel = r"Density$, \rho \,(kg / m^3)$"
            pattern = r"E=(\d+\.?\d*)"
            title = f'E = {[float(re.search(pattern, condition).group(1)) for condition in condition_names][0]}'
            title_units = title + "GPa"
        
        for i in range(2):
            for j in range(2):
                axs[i, j].set_xlabel(xlabel)
        
        fig.suptitle(title_units)
        # fig.tight_layout()  
        plt.savefig(os.path.join(general_folder, f"comparison_{title}_behavior" + str(behavior) + f".png"))
        plt.savefig(os.path.join(general_folder, f"comparison_{title}_behavior" + str(behavior) + f".eps"), format='eps')

def compare_experiments_exp3(materials:list, exp3_characteristic):
            
    # Create a dataframe 
    df = pd.DataFrame(columns=['behavior', 'condition', 'power', 'power_std', 'jerk', 'jerk_std', 'error', 'error_std', 'height', 'height_std'])
    rows = []
    
    # Loop through all experimental data and process it 
    general_folder = os.getcwd()
    exp1_folder = "Experiment_1"
    exp2_folder = "Experiment_2"
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
    plt.rcParams['figure.figsize'] = [8, 4]
    
    for behavior in range(1, 5):
        
        # Get colors based on conditions
        # for i in range(len(condition_names)):
        #     if "Increase" in condition_names[i]:
        #         condition_names[i] = "Increase"
        #     elif "Decrease" in condition_names[i]:
        #         condition_names[i] = "Decrease"
        #     else:
        #         condition_names[i] = "Material"
        
        fig, axs = make_whisker_plot(df, behavior, condition_names, labels, palette)
        
        # Set title
        title = f"{exp3_characteristic} Functional Gradient: {materials[0]}, {materials[1]}, {materials[2]}"
        title_units = title 
        fig.suptitle(title_units)
        
        # Save figure
        plt.savefig(os.path.join(general_folder,"Experiment_3", f"comparison_{title}_behavior" + str(behavior) + f".png"))
        plt.savefig(os.path.join(general_folder,"Experiment_3", f"comparison_{title}_behavior" + str(behavior) + f".eps"), format='eps')