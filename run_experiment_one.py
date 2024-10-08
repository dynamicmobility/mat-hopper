from simulation_code.helper_utils import *
from subprocess import call
import os, sys
from plotting_code.compare_experiments import *
import argparse

"""
This function runs Experiment 1.
It creates an experiment folder, generates XML files for single-link hoppers with different materials,
and runs simulations for each experiment. Finally, it compares the results.
"""

parser = argparse.ArgumentParser(description='Process materials')
parser.add_argument('--video', action='store_true', help='Flag for whether or not to save video of simulation')
args = parser.parse_args()

user_path = os.getcwd()
        
# Create main experiment folder
main_exp_folder = os.path.join(user_path, "Experiment_1")
print("Experiment Folder, " + main_exp_folder)
if not os.path.exists(main_exp_folder):
    os.makedirs(main_exp_folder)

# Determine experiment xml parameters
density_vals = [1, 1.39, 2.7, 4.43, 8]
modulus_vals = [0, 4, 70, 120, 193]
condition_names = ["MD", "PVC", "AL",  "Ti", "SS"] 
plot_labels = ["MD", "PVC", "AL",  "Ti", "SS"]
colors = [[0.901960784, 0.901960784, 0.901960784], [0.729411765, 0.635294118, 0.450980392], [0.611764706, 0.615686275, 0.709803922], [0.498039216, 0.501960784, 0.674509804], [0.411764706, 0.419607843, 0.658823529]]


# Make files
for i in range(len(density_vals)):
    density = density_vals[i]
    modulus = modulus_vals[i]
    condition_name = condition_names[i]
    
    if modulus == 0 or condition_name == "MD" or condition_name == "mujoco-default":
        cond_folder_name = os.path.join(main_exp_folder, condition_name)
        cond_folder_path = os.path.join(cond_folder_name, f'{condition_name}.xml')
        if not os.path.exists(cond_folder_name):
            os.makedirs(cond_folder_name)
        default_file_path = os.path.join(user_path, "simulation_code", "mujoco-default.xml")
        
        # Open the source file in read mode and the destination file in write mode
        with open(default_file_path, 'rb') as src, open(cond_folder_path, 'wb') as dst:
            # Read the contents from the source file and write them to the destination file
            dst.write(src.read())
    else:
        density = density_conversion(density)
        stiffness = modulus_to_stiffness(modulus)
        cond_folder_name = os.path.join(main_exp_folder, condition_name)
        if not os.path.exists(cond_folder_name):
            os.makedirs(cond_folder_name)
        cond_folder_path = os.path.join(cond_folder_name, f'{condition_name}.xml')
        base_file_path = os.path.join(user_path, "simulation_code", "base-material.xml")
        parse_file(base_file_path, cond_folder_path, str(density), str(stiffness), colors[i], colors[i], colors[i])
    
# Run Simulations
for folder in os.listdir(main_exp_folder):
    print("Current Folder: " + folder)
    folder_path = os.path.join(main_exp_folder, folder)
    if os.path.isdir(folder_path):
        xml_path = os.path.join(folder_path, folder + ".xml")
        # Run simulations for each folder
        for behavior in range(1, 5):
            pkl_name = "data_" + str(behavior) + ".pkl"
            if not pkl_name in os.listdir(folder_path):
                if sys.platform == "linux":
                    call(["python3", "simulation_code/main-hopping.py", "xml", str(behavior), xml_path, folder_path])
                else:
                    call(["mjpython", "simulation_code/main-hopping.py", "xml", str(behavior), xml_path, folder_path])   
            if args.video and not os.path.exists(os.path.join(folder_path, "behavior_" + str(behavior) + "_video.mp4")):
                call(["python", "simulation_code/record-video.py", "xml", str(behavior), xml_path, folder_path])

# Compare experiments
compare_experiments(main_exp_folder, condition_names, plot_labels)
