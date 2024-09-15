from helper_utils import *
from subprocess import call
import os, sys
from plotting_code.compare_experiments import *
from plotting_code.timeseries_plots import *
from itertools import combinations
    
def run_experiment(exp, mod_opt=None, density_opt = None):    
    """
    This function runs Experiment 3.
    It generates different permutations of density and modulus values to create `Functional Gradient' Materials and creates corresponding XML files.
    Then, it runs simulations for each experiment and compares the results.
    Args:
        exp (str): The name of the experiment.
        mod_opt (float, optional): The modulus value to use if exp is "density_incr" or "density_decr". Defaults to None.
        density_opt (float, optional): The density value to use if exp is "modulus_incr" or "modulus_decr". Defaults to None.
    """
    
    user_path = os.getcwd()
    
    # Create main experiment folder
    main_exp_folder = os.path.join(user_path, "Experiment_3")
    print("Experiment Folder, " + main_exp_folder)
    if not os.path.exists(main_exp_folder):
        os.makedirs(main_exp_folder)
    
    # Determine experiment xml parameters
    density_vals = [1.39, 2.7, 4.43, 8]
    modulus_vals = [4, 70, 120, 193]
    # condition_names = ["mujoco-default", "pvc", "aluminum", "titanium", "stainlesssteel"]
    plot_labels = ["PVC", "AL",  "Ti", "SS"] #["PVC", "AL",  "Ti", "SS"]
    density_permutations_increasing = [[i]*3 for i in density_vals]
    density_permutations_increasing.extend([list(combo) for combo in combinations(density_vals, 3)])
    density_permutations_decreasing = [[i]*3 for i in density_vals[::-1]]
    density_permutations_decreasing.extend([list(combo) for combo in combinations(sorted(density_vals, reverse=True), 3)])
    label_combinations_increasing = [[i]*3 for i in plot_labels]
    label_combinations_increasing.extend([list(plot_labels[i] for i in combo) for combo in combinations(sorted(range(len(density_vals)), key=lambda i: density_vals[i]), 3)])
    label_combinations_decreasing = [[i]*3 for i in plot_labels[::-1]]
    label_combinations_decreasing.extend([list(plot_labels[i] for i in combo) for combo in combinations(range(len(plot_labels) - 1, -1, -1), 3)])
    modulus_permutations_increasing = [[i]*3 for i in modulus_vals]
    modulus_permutations_increasing.extend([list(combo) for combo in combinations(modulus_vals, 3)])
    modulus_permutations_decreasing = [[i]*3 for i in modulus_vals[::-1]]
    modulus_permutations_decreasing.extend([list(combo) for combo in combinations(sorted(modulus_vals, reverse=True), 3)])

    # Make files
    if exp == "materials_incr":
        permutations = density_permutations_increasing
        labels = label_combinations_increasing
        modulus = modulus_permutations_increasing
        exp_folder = "Materials_Increase"
    if exp == "materials_decr":
        permutations = density_permutations_decreasing
        labels = label_combinations_decreasing
        modulus = modulus_permutations_decreasing
        exp_folder = "Materials_Decrease"
    if exp == "density_incr":
        permutations = density_permutations_increasing
        labels = label_combinations_increasing
        modulus = 70 if mod_opt is None else mod_opt
        exp_folder = "Density_Increase"
    if exp == "density_decr":
        permutations = density_permutations_decreasing
        labels = label_combinations_decreasing
        modulus = 70 if mod_opt is None else mod_opt
        exp_folder = "Density_Decrease"
    if exp == "modulus_incr":
        permutations = modulus_permutations_increasing
        labels = label_combinations_increasing
        density = 2.7 if density_opt is None else density_opt
        exp_folder = "Modulus_Increase"
    if exp == "modulus_decr":
        permutations = modulus_permutations_decreasing
        labels = label_combinations_decreasing
        density = 2.7 if density_opt is None else density_opt
        exp_folder = "Modulus_Decrease"

    for p in range(len(permutations)):
        label1 = labels[p][0]
        label2 = labels[p][1]
        label3 = labels[p][2]
        if exp == "materials_incr" or exp == "materials_decr":
            condition_name = f'{label1}_{label2}_{label3}'
        if exp == "density_decr" or exp == "density_incr":
            condition_name = f'E={modulus}, ρ={label1}_{label2}_{label3}'
        if exp == "modulus_incr" or exp == "modulus_decr":
            condition_name = f'ρ={density}, E={label1}_{label2}_{label3}'
        cond_folder_name = os.path.join(main_exp_folder, exp_folder, condition_name)
        if not os.path.exists(cond_folder_name):
            os.makedirs(cond_folder_name)
        cond_folder_path = os.path.join(cond_folder_name, f'{condition_name}.xml')
        base_file_path = os.path.join(user_path, "models", "material_single_link_hoppers", "base.xml")
        if exp == "materials_incr" or exp == "materials_decr":
            density_values = list(map(density_conversion, permutations[p]))
            modulus_values = list(map(modulus_to_stiffness, modulus[p]))
            print(labels[p], density_values, modulus_values)
            parse_file(base_file_path, cond_folder_path, density_values, modulus_values)
        if exp == "density_decr" or exp == "density_incr":
            density_values = list(map(density_conversion, permutations[p]))
            modulus_values = [modulus_to_stiffness(modulus)]*3
            parse_file(base_file_path, cond_folder_path, density_values, modulus_values)
        if exp == "modulus_incr" or exp == "modulus_decr":
            modulus_values = map(modulus_to_stiffness, permutations[p])
            density_values = [density_conversion(density)]*3
            parse_file(base_file_path, cond_folder_path, density_values, modulus_values)
    # Run Simulations

    print(os.listdir(os.path.join(main_exp_folder, exp_folder)))
    for folder in os.listdir(os.path.join(main_exp_folder, exp_folder)):
        print("Current Folder: " + folder)
        folder_path = os.path.join(main_exp_folder, exp_folder, folder)
        print(folder_path)
        if os.path.isdir(folder_path):
            xml_path = os.path.join(folder_path, folder + ".xml")
            print(xml_path)
            # Run simulations for each folder
            for behavior in range(1, 5):
                pkl_name = "data_" + str(behavior) + ".pkl"
                if not pkl_name in os.listdir(folder_path):
                    if sys.platform == "linux":
                        call(["python3", "single_leg_hopper_code/main-hopping.py", "xml", str(behavior), xml_path, folder_path])
                    else:
                        call(["mjpython", "single_leg_hopper_code/main-hopping.py", "xml", str(behavior), xml_path, folder_path])            
    
    
    
# Run all experiments
exps = ["materials_incr", "materials_decr", "density_incr", "density_decr", "modulus_incr", "modulus_decr"]
for exp in exps:
    run_experiment(exp)
    
# Plot all experiments
permutations = [list(combo) for combo in combinations(["PVC", "Ti", "SS"], 3)] #["PVC", "AL", "Ti", "SS"]
for permutation in permutations:
    for characteristic in ["Density", "Materials"]:
        pass
        compare_experiments_exp3(permutation, characteristic)
        
# Plot time series
combination = ["PVC", "Ti", "SS"]
for characteristic in ["Density", "Materials"]:
    make_timeseries_plots(combination, characteristic)