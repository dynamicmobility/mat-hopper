import time, csv, pickle
import matplotlib.pyplot as plt
from collections import defaultdict
import os

class History:
  
  def __init__(self):
    self.data = defaultdict(list)
    self.save_folder_name = 'default_results_folder'
    
  def add(self, value, key):
    self.data[key].append(value)

  def view_dict(self):
    return dict(self.data)
  
  def extract_values(self, key):
    return self.data.get(key)
  
  def save(self, filename):
    with open(filename, 'wb') as file:
      pickle.dump(self.data, file)
  
  def load_saved_dict(self, filename):
    with open(str(filename), 'rb') as file:
      loaded_dict = pickle.load(file)
      self.data = loaded_dict

  def plot_all(self, filename):
    self.load_saved_dict(filename)
    keys = list(self.data.keys())
    for key in keys:
      values = self.extract_values(key)
      # print(values[0])
      # min_time = min(values[0])
      min_time = values[0][0]
      for pair in values:
        pair[0] -= min_time
      temp_list = list(zip(*values))
      tim, vals = list(temp_list[0]), list(temp_list[1])
      plt.plot(tim, vals)
      plt.legend()
      plt.xlabel('Time')
      plt.ylabel(key)
      
      # Create a folder and save the plots
      os.makedirs(self.save_folder_name, exist_ok=True)
      plt.savefig(os.path.join(self.save_folder_name, str(key) + ".png"))
      plt.close()

  def plot_and_sort_all(self, filename):
      self.load_saved_dict(filename)
      keys = list(self.data.keys())
      for key in keys:
        values = self.extract_values(key)
        min_time = values[0][0]
        for pair in values:
          pair[0] -= min_time
        temp_list = list(zip(*values))
        tim, vals = list(temp_list[0]), list(temp_list[1])
        if key == "base_z":
          max_val = max(vals)
          new_dict = {"max_base_z": max_val, "times": tim, "values": vals}
          os.makedirs(f"{self.save_folder_name}/{key}", exist_ok=True)

          with open(f"{self.save_folder_name}/{key}/{key}_{filename}", 'wb') as file:
            pickle.dump(new_dict, file)
          
        plt.plot(tim, vals)
        plt.legend()
        plt.xlabel('Time')
        plt.ylabel(key)
        
        # Create a folder and save the plots
        os.makedirs(self.save_folder_name, exist_ok=True)
        plt.savefig(os.path.join(self.save_folder_name, str(key) + ".png"))
        plt.close()

  def plot_efficiency(self, base_z_path):
    # load each z_file
    # base_z_files = [file for file in os.listdir() if file.startswith("base_z" and file.endswith(".pkl"))]
    base_z_files = os.listdir(base_z_path)
    num_files = len(base_z_files)
    running_z_vals = 0
    for filename in base_z_files:
      with open(str(filename), 'rb') as file:
        loaded_dict = pickle.load(file)
        running_z_vals += loaded_dict.get("max_base_z")
        # self.data = loaded_dict
    return running_z_vals/num_files

    #  pull out the max_base_z value
    

    # plot max(base_z) for all files

    