from simulation_code.helper_utils import *
from subprocess import call
import os, sys
from plotting_code.compare_experiments import *
import re
import argparse

"""
This function runs Experiment 2 based on the given experiment name.
It creates an experiment folder, generates XML files with different density and modulus values,
and runs simulations for each experiment. Finally, it compares the results.
"""

def plot_exp2(experiment_name):
    user_path = os.getcwd()
    main_exp_folder = os.path.join(user_path, "Experiment_2", experiment_name)
    if experiment_name == "PVC_Sweep":
        density_vals = sorted([1.39, 1.10, 1.45, 1.90, 2.5])
        modulus_vals =  sorted([4, 0.1, 0.45, 2.01, 9.])
        base_density = [1.39]
        base_modulus = [4]
    if experiment_name == "AL_Sweep":
        density_vals  = sorted([2.7, 3.95, 8.66])
        modulus_vals = sorted([70, 20. ,  62.14])
        base_density = [2.7]
        base_modulus = [70]
    if experiment_name == "Ti_Sweep":
        density_vals   = sorted([4.43, 2.00, 4.24, 8.97, 19.00])
        modulus_vals = sorted([120, 18.        ,  63.76, 225.85, 800.        ])
        base_density = [4.43]
        base_modulus = [120]
    if experiment_name == "SS_Sweep":
        density_vals  = sorted([8, 2, 4.31, 9.28, 20])
        modulus_vals = sorted([193, 15.        ,   60.82,  246.62, 1000.        ])
        base_density = [8]
        base_modulus = [193]
    condition_names = []
    
    # Make files
    for m in range(len(modulus_vals)):
        condition_name = f"\u03C1={base_density[0]}, E={modulus_vals[m]}"
        condition_names.append(condition_name)
        
    # Density sweep (same density, diff E)
    cond_1 = [condition_names[i] for i in range(len(condition_names)) if condition_names[i].startswith(f"\u03C1={base_density[0]},")] #[:4]
    pattern = pattern = r"E=(\d+\.?\d*)"
    labels = [float(re.search(pattern, condition).group(1)) for condition in cond_1]
    compare_experiments_exp2(main_exp_folder, cond_1, labels, density=True)
    # heatmaps(main_exp_folder, cond_1, labels)
    
    condition_names = []
    for i in range(len(density_vals)):
        condition_name = f"\u03C1={density_vals[i]}, E={base_modulus[0]}"
        condition_names.append(condition_name)
        
    # Modulus Sweep: same modulus, diff rho
    cond_2 = [condition_names[i] for i in range(len(condition_names)) if condition_names[i].endswith(f"E={base_modulus[0]}")] 
    pattern = r"ρ=(\d+\.?\d*)"
    labels = [float(re.search(pattern, condition).group(1)) for condition in cond_2]
    compare_experiments_exp2(main_exp_folder, cond_2, labels, density=False)

def run_exp(experiment_name, video_flag):
    user_path = os.getcwd()
    main_exp_folder = os.path.join(user_path, "Experiment_2", experiment_name)
    if not os.path.exists(main_exp_folder):
        os.makedirs(main_exp_folder)
            
    if experiment_name == "PVC_Sweep":
        density_vals = sorted([1.39, 1.10, 1.45, 1.90, 2.5])
        modulus_vals =  sorted([4, 0.1, 0.45, 2.01, 9.])
        base_density = [1.39]
        base_modulus = [4]
        base = "pvc"
        acr = "PVC"
        color = [0.729411765, 0.635294118, 0.450980392]
    elif experiment_name == "AL_Sweep":
        density_vals  = sorted([2.7, 3.95, 8.66])
        modulus_vals = sorted([70, 20. ,  62.14])
        base_density = [2.7]
        base_modulus = [70]
        base = "aluminum"
        acr = "AL"
        color = [0.611764706, 0.615686275, 0.709803922]

    elif experiment_name == "Ti_Sweep":
        density_vals   = sorted([4.43, 2.00, 4.24, 8.97, 19.00])
        modulus_vals = sorted([120, 18.        ,  63.76, 225.85, 800.        ])
        base_density = [4.43]
        base_modulus = [120]
        base = "titanium"
        acr = "Ti"
        color = [0.498039216, 0.501960784, 0.674509804]
    elif experiment_name == "SS_Sweep":
        density_vals  = sorted([8, 2, 4.31, 9.28, 20])
        modulus_vals = sorted([193, 15.        ,   60.82,  246.62, 1000.        ])
        base_density = [8]
        base_modulus = [193]
        base = "stainlesssteel"
        acr = "SS"
        color = [0.411764706, 0.419607843, 0.658823529]
    else:
        raise ValueError("Invalid experiment name. Must be either 'PVC_Sweep', 'AL_Sweep', 'Ti_Sweep', or 'SS_Sweep'")
    
    # Make files
    condition_names = []
    for m in range(len(modulus_vals)):
        density = base_density[0]
        modulus = modulus_vals[m]
        condition_name = f"\u03C1={density}, E={modulus_vals[m]}"
        condition_names.append(condition_name)
        
        density = density_conversion(density)
        stiffness = modulus_to_stiffness(modulus)
        cond_folder_name = os.path.join(main_exp_folder, condition_name)
        if not os.path.exists(cond_folder_name):
            os.makedirs(cond_folder_name)
            
        cond_folder_path = os.path.join(cond_folder_name, f'{condition_name}.xml')
        base_file_path = os.path.join(user_path, "simulation_code", "base-material.xml")
        parse_file(base_file_path, cond_folder_path, str(density), str(stiffness), color, color, color)
        
        
    for i in range(len(density_vals)):
        density = density_vals[i]
        modulus = base_modulus[0]
        condition_name = f"\u03C1={density_vals[i]}, E={modulus}"
        condition_names.append(condition_name)
        
        density = density_conversion(density)
        stiffness = modulus_to_stiffness(modulus)
        cond_folder_name = os.path.join(main_exp_folder, condition_name)
        if not os.path.exists(cond_folder_name):
            os.makedirs(cond_folder_name)
            
        cond_folder_path = os.path.join(cond_folder_name, f'{condition_name}.xml')
        base_file_path = os.path.join(user_path, "simulation_code", "base-material.xml")
        parse_file(base_file_path, cond_folder_path, str(density), str(stiffness), color, color, color)
        
    # Run Simulations
    for folder in os.listdir(main_exp_folder):
        print("Current Folder: " + folder)
        folder_path = os.path.join(main_exp_folder, folder)
        if os.path.isdir(folder_path):
            xml_path = os.path.join(folder_path, folder + ".xml")
            # Run simulations for each folder
            for behavior in [2]: #range(1, 5):
                pkl_name = "data_" + str(behavior) + ".pkl"
                if not pkl_name in os.listdir(folder_path):
                    if sys.platform == "linux":
                        call(["python3", "simulation_code/main-hopping.py", "xml", str(behavior), xml_path, folder_path])
                    else:
                        call(["mjpython", "simulation_code/main-hopping.py", "xml", str(behavior), xml_path, folder_path])          
                        
                if video_flag and not os.path.exists(os.path.join(folder_path, "behavior_" + str(behavior) + "_video.mp4")):
                    call(["python", "simulation_code/record-video.py", "xml", str(behavior), xml_path, folder_path])  
                        
    # Plot Experiments
    plot_exp2(experiment_name)

# MAIN FUNCTION CALL

parser = argparse.ArgumentParser(description='Process materials')
parser.add_argument('--video', action='store_true', help='Flag for whether or not to save video of simulation')
args = parser.parse_args()

exps = ["PVC_Sweep", "AL_Sweep"]
for exp in exps:
    run_exp(exp, args.video)
