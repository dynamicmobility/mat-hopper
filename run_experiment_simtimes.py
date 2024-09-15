import numpy as np
from simulation_code.history import *
from subprocess import call
import os
from simulation_code.helper_utils import *
from plotting_code.compare_simtimes import compare_dts

experiment_name = "Simulation Speed"
user_path = os.getcwd()
num_steps = 20 # Number of points per axis for grid

# Create main experiment folder
main_exp_folder = os.path.join(user_path, experiment_name)
print("Experiment Folder, " + main_exp_folder)
if not os.path.exists(main_exp_folder):
    os.makedirs(main_exp_folder)
    
# Determine experiment xml parameters
density_range = [1.39,  8]
stiffness_range = [4, 340]
density_vals = np.round(np.logspace(np.log10(density_range[0]), np.log10(density_range[1]), num=num_steps), 1)
stiffness_vals = np.round(np.logspace(np.log10(stiffness_range[0]), np.log10(stiffness_range[1]), num=num_steps), 1)

# Get all combinations and make xml files
for i in range(num_steps):
    for j in range(num_steps):
        density = density_conversion(density_vals[i])
        stiffness = modulus_to_stiffness(stiffness_vals[j])
        condition_name = f'combination_{i}_{j}'
        cond_folder_name = os.path.join(main_exp_folder, condition_name)
        if not os.path.exists(cond_folder_name):
            os.makedirs(cond_folder_name)
            
        cond_folder_path = os.path.join(cond_folder_name, f'{condition_name}.xml')
        base_file_path = os.path.join(user_path, "simulation_code", "base-material.xml")
        parse_file(base_file_path, cond_folder_path, str(density), str(stiffness))

# Run Simulations with decreasing dt until convergence
for folder in os.listdir(main_exp_folder):
    folder_path = os.path.join(main_exp_folder, folder)
    if os.path.isdir(folder_path):
        xml_path = os.path.join(folder_path, folder + ".xml")
        temp_xml_path = os.path.join(folder_path, folder + "_dt.xml")
        # Run simulations for each folder
        behavior = 1
        success_name = "success.txt"
        
        # dt = 0.1
        dt_vec = np.logspace(np.log10(0.001), np.log10(0.00001), num=20)
        # dt_vec = np.linspace(0.001, 0.000001, num=10)
        i = 0
        while not success_name in os.listdir(folder_path):
            dt = dt_vec[i]
            parse_file_dt(xml_path, temp_xml_path, str(dt))
            call(["mjpython", "simulation_code/test-failure.py", "xml", str(behavior), temp_xml_path, folder_path])   
            dt = dt*0.1
            i = i + 1

compare_dts(main_exp_folder, density_vals, stiffness_vals, dt_vec)