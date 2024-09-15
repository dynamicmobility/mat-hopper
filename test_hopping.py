from subprocess import call
import os, math, sys


behavior = 3
xml_path = 'simulation_code/mujoco-default.xml'
folder_path = 'test_sim_results'

call(["mjpython", "simulation_code/main-hopping.py", "xml", str(behavior), xml_path, folder_path])        