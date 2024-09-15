import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from single_leg_hopper_code.history import *
import os
from matplotlib.ticker import ScalarFormatter

def compare_dts(general_folder, density_vals, stiffness_vals, dt_vals):

    data = []
    X, Y = np.meshgrid(density_vals, stiffness_vals)
    Z = np.zeros((len(density_vals), len(stiffness_vals)))
    num_steps = len(density_vals)
    # Get all combinations and make xml files
    for i in range(num_steps):
        for j in range(num_steps):
            condition_name = f'combination_{i}_{j}'
            cond_folder_name = os.path.join(general_folder, condition_name)
            
            # Get dt value from success.txt file
            success_file_path = os.path.join(cond_folder_name, "success.txt")
            with open(success_file_path, "r") as success_file:
                dt = float(success_file.readline().strip())
                
            # Purely for santity sake:
            X[i, j] = np.log10(density_vals[i])
            Y[i, j] = np.log10(stiffness_vals[j])
            Z[i, j] = dt
                
            data.append({'density': density_vals[i],
                        'stiffness': stiffness_vals[j],
                        'dt': dt})
            
    plt.rcParams.update({'font.size': 10})
    plt.rcParams.update({'font.family': 'Palatino Linotype', 'text.usetex': True})
    
    fig, ax = plt.subplots()
    
    # im = ax.scatter(X, Y, Z, norm=LogNorm(vmin=dt_vals[0], vmax=dt_vals[-1]), cmap="gray")
    # im = ax.scatter(X, Y, c=Z, norm=LogNorm(vmin=dt_vals[0], vmax=dt_vals[-1]), cmap="gray")
    # im = ax.pcolormesh(X, Y, Z, norm=LogNorm(vmin=dt_vals[0], vmax=dt_vals[-1]), cmap="gray")
    im = ax.pcolormesh(X, Y, Z, norm=LogNorm(vmin=dt_vals[0], vmax=dt_vals[-1]), cmap="PuOr")
    # ax.grid(color='white', linestyle='-', linewidth=0.5)
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("Time Step $(s)$")
    
    ax.set_xlabel("Density (g/cm$^3$)")
    ax.set_xticks(np.log10(density_vals[::2]))
    ax.set_xticklabels(density_vals[::2])
    
    ax.set_ylabel("Modulus (GPa)")
    ax.set_yticks(np.log10(stiffness_vals[::2]))
    ax.set_yticklabels(stiffness_vals[::2])
   
    # ADD KNOWN MATERIALS TO THE PLOT
    material_modulus = np.log10([4, 70, 120, 193, 340])
    material_density = np.log10([1.39, 2.7, 4.43, 8, 3.9])
    material_name = ["   PVC", " AL", " TA", " SS", " Al-Ox"]
    for i in range(len(material_modulus)):
        ax.scatter(material_density[i], material_modulus[i], color='black', marker='x', s=50)
        # ax.text(material_density[i], material_modulus[i], material_name[i], fontsize=12, ha='left', va='top', bbox=dict(facecolor='white', edgecolor='black', pad=5))
    
    plt.savefig(os.path.join(general_folder, "heatmap.png"))
    plt.savefig(os.path.join(general_folder, "heatmap.eps"), format='eps')
    