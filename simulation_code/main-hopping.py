import time
import mujoco
import mujoco.viewer
import math
import numpy as np
import itertools
import os
import argparse

from control_utils import get_torques, x_base_position_control, y_base_position_control
from hopper import Hopper 
from history import History
from helper_utils import make_basic_materials

hopper = Hopper()
history = History()

parser = argparse.ArgumentParser(description='Process materials')

parser.add_argument('material', nargs="?", help='options: [aluminum, pvc, titanium, stainlesssteel, alumoxide]', choices=["aluminum", "pvc", "titanium", "stainlesssteel", "alum-oxide", "xml"])
parser.add_argument('behavior', nargs=1, help='1 for hopping, 2 for forward, 3 for ramp, 4 for circle', choices=["1", "2", "3", "4"])
parser.add_argument('xml_file', nargs="?", help="path to xml file, eg: 'simulation_code/mujoco-default.xml")
parser.add_argument('save_folder', nargs="?", help="path to save folder, eg: 'results/aluminum/'")
args = parser.parse_args()

# OPTIONS OF MATERIALS
if args.material:
  make_basic_materials()
  if args.material[0] == "aluminum":
    m = mujoco.MjModel.from_xml_path('../models/material_single_link_hoppers/aluminum.xml')
  elif args.material[0] == "pvc":
    m = mujoco.MjModel.from_xml_path('../models/material_single_link_hoppers/pvc.xml')
  elif args.material[0] == "titanium":
    m = mujoco.MjModel.from_xml_path('../models/material_single_link_hoppers/titanium.xml')
  elif args.material[0] == "stainlesssteel":
    m = mujoco.MjModel.from_xml_path('../models/material_single_link_hoppers/stainlesssteel.xml')
  elif args.material[0] == "alum-oxide":
    m = mujoco.MjModel.from_xml_path('../models/material_single_link_hoppers/alum-oxide.xml')
if args.xml_file:
  m = mujoco.MjModel.from_xml_path(args.xml_file)
  print("Using xml file: ", args.xml_file)
  print(m)
else:
  print("no mujoco model indicated")
  
# ASSIGN BEHAVIOR AND MATERIAL PROPERTIES
behavior = int(args.behavior[0]) # 1 for hopping, 2 for forward, 3 for ramp, 4 for circle

# GET RID OF RAMP IF NOT RAMP BEHAVIOR
if behavior != 3:
  m.geom('ramp').conaffinity = 0
  m.geom('ramp').rgba = [1, 1, 1, 0]
  
# Load in MuJoCo Data Object
d = mujoco.MjData(m)

# Set initial position and velocity
d.qpos[0] = 0  # x position of base
d.qpos[1] = 0  # y position of base
d.qpos[2] = 1.2  # z position of base
d.qpos[3] = 0  # roll (rotation around x-axis) of base
d.qpos[4] = 0  # pitch (rotation around y-axis) of base
d.qpos[5] = 0  # yaw (rotation around z-axis) of base
d.qpos[6] = 0  # hip rotation around y-axis (sagittal plane)
d.qpos[7] = 0  # hip rotation around x-axis (frontal plane)

# Start Hopper based on behavior
if behavior == 4:
  d.qpos[0] = 0.5
    
with mujoco.viewer.launch_passive(m, d) as viewer:

  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 60:
    
    # For time keeping
    step_start = time.time()
  
    # Update hopper class
    hopper.update_actual(d)
    
    # Translate global position to local base frame
    t = time.time() - start  # Current simulation time
    commanded_vel = 0.05
    if behavior == 1:
      x_des_global = 0
      y_des_global = 0
      x_dot_des_global = 0
      y_dot_des_global = 0
    elif behavior == 2 or behavior == 3:
      x_des_global = commanded_vel*t
      y_des_global = 0
      x_dot_des_global = commanded_vel
      y_dot_des_global = 0
    elif behavior == 4:
      # Generate x_des_global and y_des_global as a circle with radius 0.5 and period of 10 seconds
      t = time.time() - start  # Current simulation time
      radius = 0.5
      period = 2*math.pi*radius/commanded_vel #240 # period dictated by how fast the hopper can go
      angular_velocity = 2 * math.pi / period  # Angular velocity in radians per second
      x_des_global = radius * math.cos(angular_velocity * t)
      y_des_global = radius * math.sin(angular_velocity * t)
      x_dot_des_global = -radius * angular_velocity * math.sin(angular_velocity * t)
      y_dot_des_global = radius * angular_velocity * math.cos(angular_velocity * t)
    hopper.convert_pos_to_local_frame(x_des_global, y_des_global,x_dot_des_global,y_dot_des_global)
    
    # Add Marker for Desired Position
    viewer.user_scn.ngeom = 1
    geom_id = 0
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[geom_id],
        # name = 'desired_base_position',
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.02, 0, 0],
        pos=np.array([x_des_global, y_des_global, 0]),
        mat=np.eye(3).flatten(),
        rgba=0.5*np.array([0, 0, 1, 2])
    )
    geom_id += 1
    viewer.sync()
    
    # Determine Desired Foot Position and Velocity
    x_foot_des = x_base_position_control(hopper)
    hipy_des, hipy_dot_des = hopper.get_desired_hipy_state(x_foot_des, 0)

    y_foot_des = y_base_position_control(hopper)
    hipx_des, hipx_dot_des = hopper.get_desired_hipx_state(y_foot_des, 0)
      
    # Apply PD Control with different gains for crouching vs extending
    hipy_torque, hipx_torque = get_torques(hopper)
    
    # Fix wierd runtime errors that sometimes happen with the torques not being assigned to scalar values
    if not isinstance(hipy_torque, (int, float)):
      hipy_torque = hipy_torque[0]
    if not isinstance(hipx_torque, (int, float)):
      hipx_torque = hipx_torque[0]
    
    # Attutude Control applied during stance and foot position control applied during flight
    if hopper.sensor['grf'] > 0:
      # print("attitude")      
      d.ctrl[0] = 4*(hopper.actual['pitch_base']) + 1*(hopper.actual['pitch_base_dot'])
      d.ctrl[1] = 4*(hopper.actual['roll_base']) + 1*(hopper.actual['roll_base_dot'])
    else:
      # print("flight")
      d.ctrl[0] = hipy_torque
      d.ctrl[1] = hipx_torque
      
    # Apply Thrust if GRF is high enough 
    if hopper.sensor['grf'] > 0 and hopper.actual['z_base_dot'] > 0.01:
      # print('Adding Thrust')
      d.ctrl[2] = hopper.hop_history['adapted_hop_torque']
    else:
      # print("No Thrust")
      d.ctrl[2] = 0
    
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Execute things that only happen every 0.1 seconds
    if (time.time()-start) % 0.1 < 0.0001:
      # print('time: ',time.time()-start)
      history.add(time.time()-start,'t')
      history.add(hopper.sensor['base_acc'][0],'base_acc_x')
      history.add(hopper.sensor['base_acc'][1],'base_acc_y')
      history.add(hopper.sensor['base_acc'][2],'base_acc_z')
      history.add(hipy_torque,'hipy_torque')
      history.add(hipx_torque,'hipx_torque')
      history.add(hopper.actual['hipy_dot'],'hipy_dot')
      history.add(hopper.actual['hipx_dot'],'hipx_dot')
      history.add(hopper.actual['x_base'],'x_base')
      history.add(hopper.actual['y_base'],'y_base')
      history.add(hopper.actual['z_base'],'z_base')
      history.add(hopper.desired['x_base'],'x_base_des')
      history.add(hopper.desired['y_base'],'y_base_des')
      history.add(hopper.state.value ,'state')
      history.add(hopper.sensor['grf'],'grf')
      
      # Example modification of a viewer option: toggle contact points every two seconds.
      with viewer.lock():
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = m.body('base').id
        # viewer.cam.elevation = -30    # Can adjust the camera angle
        # viewer.cam.distance = 3       # Can adjust the camera distance

      # Pick up changes to the physics state, apply perturbations, update options from GUI.
      viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
      
print("SAVING DATA")
save_folder = args.save_folder
if not os.path.exists(save_folder):
  os.makedirs(save_folder)
history.save(os.path.join(save_folder,f'data_{behavior}.pkl'))
      
    
