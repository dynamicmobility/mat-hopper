from subprocess import call
import os, math, sys


behavior = 3
xml_path = 'simulation_code/mujoco-default.xml'
folder_path = 'test_sim_results'
video_flag = False

if video_flag:
    call(["python", "simulation_code/record-video.py", "xml", str(behavior), xml_path, folder_path])        
else:
    call(["mjpython", "simulation_code/main-hopping.py", "xml", str(behavior), xml_path, folder_path])