import time
import mujoco
import mujoco.viewer
import math
import numpy as np
import itertools
import os
import argparse
import glfw
from subprocess import call

from control_utils import get_torques, x_base_position_control, y_base_position_control
from hopper import Hopper 
from history import History
from helper_utils import make_basic_materials

# Initialize GLFW for offscreen rendering
def init_glfw(width, height):
    # Initialize GLFW
    if not glfw.init():
        raise Exception("Could not initialize GLFW")

    # Create invisible window for offscreen rendering
    # glfw.window_hint(glfw.VISIBLE, glfw.TRUE)
    # glfw.window_hint(glfw.DOUBLEBUFFER, glfw.FALSE)
    
    print("Setting window to be: ", width, height)
    window = glfw.create_window(width, height, "Offscreen", None, None)
    if not window:
        glfw.terminate()
        raise Exception("Could not create GLFW window")

    glfw.make_context_current(window)
    window_width, window_height = glfw.get_window_size(window)
    print("Actual Window Size: ", window_width, window_height)
    return window

# Clean up GLFW and MuJoCo resources
def close_mujoco_glfw(window, context, scene):
    # Destroy the GLFW window and terminate
    glfw.destroy_window(window)
    glfw.terminate()
    
    # Clean up MuJoCo context and scene (Python handles memory)
    del context
    del scene
    
# Initialize Hopper and History Classes
hopper = Hopper()
history = History()

fast_option = False

# Parse through function arguments
parser = argparse.ArgumentParser(description='Process materials')
parser.add_argument('material', nargs="?", help='options: [aluminum, pvc, titanium, stainlesssteel, alumoxide]', choices=["aluminum", "pvc", "titanium", "stainlesssteel", "alum-oxide", "xml"])
parser.add_argument('behavior', nargs=1, help='1 for hopping, 2 for forward, 3 for ramp, 4 for circle', choices=["1", "2", "3", "4"])
parser.add_argument('xml_file', nargs="?", help="path to xml file, eg: 'simulation_code/mujoco-default.xml")
parser.add_argument('save_folder', nargs="?", help="path to save folder, eg: 'results/aluminum/'")
args = parser.parse_args()

# Speed up hopping speed
if fast_option:
    hopper.params['max_velocity'] = 0.3
else:
    hopper.params['max_velocity'] = 0.1

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
#   print(m)
else:
  print("no mujoco model indicated")
  
# ASSIGN BEHAVIOR AND MATERIAL PROPERTIES
behavior = int(args.behavior[0]) # 1 for hopping, 2 for forward, 3 for ramp, 4 for circle

# GET RID OF RAMP IF NOT RAMP BEHAVIOR
if behavior != 3:
  m.geom('ramp').conaffinity = 0
  m.geom('ramp').rgba = [1, 1, 1, 0]
  
# m.geom('pos_marker').pos = [0.5, 0, 0]
# m.geom('pos_marker').rgba = [0, 0, 0, 1]

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
    
# Video Settings   
duration = 60                  # Duration of simulation
fps = 60                       # Frames per second
output_file = "output.rgb"      #Temporary RGB output file
add_depth = False                # Add depth information to the output

 # Initialize OpenGL and MuJoCo
# Get the primary monitor's resolution
width, height = 2160, 2160 # (would like 1920 x 1080)
window = init_glfw(width, height)

# Create visualization structures
scene = mujoco.MjvScene(m, maxgeom=2000)
context = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_150)
mujoco.mjr_resizeOffscreen(width, height, context)

# Initialize camera and option
cam = mujoco.MjvCamera()
# cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
# cam.trackbodyid = m.body('base').id

cam.elevation = -25
cam.distance = 2.5
cam.lookat[2] = 0.5
opt = mujoco.MjvOption()

# mujoco.mjv_addGeoms(m, d, opt, None, mujoco.mjtCatBit.mjCAT_ALL, scene)

# Set offscreen rendering
mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)

# Allocate RGB and depth buffers
rgb_buffer = np.zeros((height, width, 3), dtype=np.uint8)
depth_buffer = np.zeros((height, width), dtype=np.float32)

# Open output file
with open(output_file, "wb") as f:
    frametime = 0
    frame_count = 0
        
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while time.time() - start < duration:
        
        if (d.time - frametime) > (1 / fps) or frametime == 0:
            
            print("Time: ", time.time() - start)
        
            # For time keeping
            step_start = time.time()

            # Update scene
            cam.lookat[0] = d.qpos[0] # update x-axis of camera to follow hopper
            mujoco.mjv_updateScene(m, d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
             
            # Cycle through RGB colors continuously based on time
            # cycle_time = 0.5  # Time in seconds for a full cycle through RGB colors
            # elapsed_time = time.time() - start
            # color_phase = (elapsed_time % cycle_time) / cycle_time  # Normalized phase in the range [0, 1]

            # Calculate RGB values based on the phase
            # r = max(0, min(1, 2 * (1 - abs(3 * color_phase - 1))))
            # g = max(0, min(1, 2 * (1 - abs(3 * color_phase - 2))))
            # b = max(0, min(1, 2 * (1 - abs(3 * color_phase - 3))))
            # r = hopper.params['leg_color'][0]
            # g = hopper.params['leg_color'][1]
            # b = hopper.params['leg_color'][2]

            # # Apply the calculated color to the material
            # m.geom('torso').rgba = [r, g, b, 1]
            # m.geom('torso2').rgba = [r, g, b, 1]
            # m.geom('leg').rgba = [r, g, b, 1]
            # m.geom('mat1_1').rgba = [r, g, b, 1]
            # m.geom('mat1_2').rgba = [r, g, b, 1]
            # m.geom('mat2_1').rgba = [r, g, b, 1]
            # m.geom('mat2_2').rgba = [r, g, b, 1]
            # m.geom('mat3_1').rgba = [r, g, b, 1]
            # m.geom('mat3_2').rgba = [r, g, b, 1]
            # m.geom('foot_upper').rgba = [r, g, b, 1]
            # m.geom('foot_lower').rgba = [r, g, b, 1]
  
            # Create a MuJoCo viewport (MjrRect) for rendering within the window
            # Position it at (0, 0) and make it as large as the window
            viewport = mujoco.MjrRect(0, 0, width, height)

            mujoco.mjr_render(viewport, scene, context)

            # Read pixel buffers
            mujoco.mjr_readPixels(rgb_buffer, depth_buffer, viewport, context)
    
            # Write RGB data to file
            f.write(rgb_buffer.tobytes())
            
            # Print progress every 10 frames
            frame_count += 1
            # if frame_count % 10 == 0:
            #     print(".", end="", flush=True)

            # Update time for next frame
            frametime = d.time
                
        # Update hopper class
        hopper.update_actual(d)
        
        # Translate global position to local base frame
        t = time.time() - start  # Current simulation time
        if fast_option:
            commanded_vel = 0.2
            circle_vel = 0.08
        else:
            commanded_vel = 0.05
            circle_vel = 0.05
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
            period = 2*math.pi*radius/circle_vel
            angular_velocity = 2 * math.pi / period  # Angular velocity in radians per second
            x_des_global = radius * math.cos(angular_velocity * t)
            y_des_global = radius * math.sin(angular_velocity * t)
            x_dot_des_global = -radius * angular_velocity * math.sin(angular_velocity * t)
            y_dot_des_global = radius * angular_velocity * math.cos(angular_velocity * t)
        hopper.convert_pos_to_local_frame(x_des_global, y_des_global,x_dot_des_global,y_dot_des_global)
        
        # # Add Marker for Desired Position
        # Find geom 'sphere' and modify the position
        # print("Geom Name", m.geom('pos_marker').name)
        # geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'pos_marker')
        # m.geom_pos[geom_id] = [x_des_global, y_des_global, 0]
        # scene.geoms[2].pos = np.array([x_des_global, y_des_global, 0])
        # scene.geoms[2].type = mujoco.mjtGeom.mjGEOM_SPHERE
        # scene.geoms[2].size = np.array([0.02]) 
        # scene.geoms[2].rgba = np.array([1, 0, 0, 1])    
        # print(scene.geoms)
        
        # d.geom('pos_marker').pos = [x_des_global, y_des_global, 0]

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

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
      
# Release the video writer
close_mujoco_glfw(window, context, scene)

 # Convert video
if os.path.exists('video.mp4'):
    os.remove('video.mp4')
ffmpeg_command = ' '.join([
    'ffmpeg', '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', f'{width}x{height}', 
    '-framerate', f'{fps}', '-i', 'output.rgb', '-vf', 'vflip,format=yuv420p', 'video.mp4'
])
call(ffmpeg_command, shell=True)

# Move the video file to the save folder
save_folder = args.save_folder
if not os.path.exists(save_folder):
  os.makedirs(save_folder)
if fast_option:
    new_name = f'fast_behavior_{behavior}_video.mp4'
else:
    new_name = f'behavior_{behavior}_video.mp4'
video_output_path = os.path.join(save_folder, new_name)
print('new_video_path: ', video_output_path)
os.rename('video.mp4', video_output_path) 

      
    
