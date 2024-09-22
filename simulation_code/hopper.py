import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from enum import Enum
    
class State(Enum):
  FLIGHT = 1
  STANCE = 2
  
# hopper.py
class Hopper:
  def __init__(self):
    self.actual = {
      'x_base': 0,
      'y_base': 0,
      'z_base': 0,
      'roll_base': 0,
      'pitch_base': 0,
      'yaw_base': 0,
      'hipy': 0,
      'hipx': 0,
      'x_base_dot': 0,
      'y_base_dot': 0,
      'z_base_dot': 0,
      'roll_base_dot': 0,
      'pitch_base_dot': 0,
      'yaw_base_dot': 0,
      'hipy_dot': 0,
      'hipx_dot': 0,
      'x_base_local': 0,
      'y_base_local': 0,
      'x_base_dot_local': 0,
      'y_base_dot_local': 0
    }
    self.desired = {
      'x_base': 0,
      'y_base': 1,
      'z_base': 2,
      'roll_base': 3,
      'pitch_base': 4,
      'yaw_base': 5,
      'hipy': 6,
      'hipx': 7,
      'x_base_dot': 8,
      'y_base_dot': 9,
      'z_base_dot': 10,
      'roll_base_dot': 11,
      'pitch_base_dot': 12,
      'yaw_base_dot': 13,
      'hipy_dot': 14,
      'hipx_dot': 15,
      'x_base_local': 16,
      'y_base_local': 17,
      'x_base_dot_local': 0,
      'y_base_dot_local': 0
    }
    self.sensor = {
      'grf': 0,
      'base_pos': [1, 2, 3],
      'base_quat': [4, 5, 6, 7],
      'base_gyro': [8, 9, 10],
      'base_vel': [11, 12, 13],
      'hip_torque': [14,15,16],
      'base_acc': [17, 18, 19]
    }
    self.params = {
      'first_hop_flag': True,
      'leg_length': 0.61,
      'max_vel': 0.1,
      'base_hop_torque': 200,
      'desired_hop_height': 1.0,
      'leg_color': [0, 0, 0, 1],
      'leg_color_phase': 0
    }
    self.hop_history = {
      'temp_max_height': 0,
      'last_hop_height': 0, 
      'T_stance': 0.1,
      'time_of_switch': 0,
      'adapted_hop_torque': self.params['base_hop_torque']
    }
    
    self.state = State.FLIGHT
    self.last_hop_height = 0
    self.temp_max_hop_height = self.params['desired_hop_height']
    
    self.actual['roll_base'] = 0
    self.actual['pitch_base'] = 0
    self.actual['yaw_base'] = 0
    
  def update_actual(self, d):
    # Get raw sensor values
    self.sensor['grf'] = d.sensordata[0]
    self.sensor['base_pos'] = d.sensordata[1:4]
    self.sensor['base_quat'] = d.sensordata[4:8]
    self.sensor['base_gyro'] = d.sensordata[8:11]
    self.sensor['base_vel'] = d.sensordata[11:14]  
    self.sensor['hip_torque'] = d.sensordata[14:17]  
    self.sensor['base_acc'] = d.sensordata[17:20]  
    
    # Process some sensor information
    roll_x, pitch_y, yaw_z = self.quaternion_to_euler(self.sensor['base_quat'])

    # Assign values to actual dictionary
    self.actual['x_base'] = self.sensor['base_pos'][0]
    self.actual['y_base'] = self.sensor['base_pos'][1]
    self.actual['z_base'] = self.sensor['base_pos'][2]
    self.actual['roll_base'] =  roll_x
    self.actual['pitch_base'] = pitch_y # eulxyz[1]
    self.actual['yaw_base'] = yaw_z
    self.actual['hipy'] = d.qpos[6]
    self.actual['hipx'] = d.qpos[7]
    self.actual['x_base_dot'] = self.sensor['base_vel'][0]
    self.actual['y_base_dot'] = self.sensor['base_vel'][1]
    self.actual['z_base_dot'] = self.sensor['base_vel'][2]
    self.actual['roll_base_dot'] = self.sensor['base_gyro'][0]
    self.actual['pitch_base_dot'] = self.sensor['base_gyro'][1]
    self.actual['yaw_base_dot'] = self.sensor['base_gyro'][2]
    self.actual['hipy_dot'] = d.qvel[6]
    self.actual['hipx_dot'] = d.qvel[7]
    
    self.update_state_machine(d.time)
    if self.actual['z_base'] > self.hop_history['temp_max_height']:
      self.hop_history['temp_max_height'] = self.actual['z_base']
    
  def quaternion_to_euler(self, q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    # w, x, y, z = q
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]
    
    # Calculate roll angle (rotation around x-axis)
    roll_x = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
  
    # Calculate pitch angle (rotation around y-axis)
    pitch_y = math.asin(2 * (w * y - z * x))
  
    # Calculate yaw angle (rotation around z-axis)
    yaw_z = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
  
    return roll_x, pitch_y, yaw_z # in radians  
  
  
  def euler_to_quat(self, roll, pitch, yaw):
    """
    Convert euler angles (roll, pitch, yaw) into a quaternion
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    # Calculate the half angles
    roll_half = roll / 2
    pitch_half = pitch / 2
    yaw_half = yaw / 2

    # Calculate the sin and cos of the half angles
    sin_roll_half = math.sin(roll_half)
    cos_roll_half = math.cos(roll_half)
    sin_pitch_half = math.sin(pitch_half)
    cos_pitch_half = math.cos(pitch_half)
    sin_yaw_half = math.sin(yaw_half)
    cos_yaw_half = math.cos(yaw_half)

    # Calculate the quaternion components
    w = cos_roll_half * cos_pitch_half * cos_yaw_half + sin_roll_half * sin_pitch_half * sin_yaw_half
    x = sin_roll_half * cos_pitch_half * cos_yaw_half - cos_roll_half * sin_pitch_half * sin_yaw_half
    y = cos_roll_half * sin_pitch_half * cos_yaw_half + sin_roll_half * cos_pitch_half * sin_yaw_half
    z = cos_roll_half * cos_pitch_half * sin_yaw_half - sin_roll_half * sin_pitch_half * cos_yaw_half

    return [w, x, y, z]
  
  def convert_pos_to_local_frame(self, x_des_global, y_des_global, x_dot_des_global, y_dot_des_global):
    yaw = self.actual['yaw_base']
    
    # Transformation matrix from local to world frame
    R_local_to_world = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw),  np.cos(yaw)]
    ])
    T_local_to_world = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, self.actual['x_base']],
        [np.sin(yaw),  np.cos(yaw), 0, self.actual['y_base']],
        [0,            0,           1, self.actual['z_base']],
        [0,            0,           0, 1]
    ])
    
    # Inverse transformation matrix from world to local frame
    T_world_to_local = np.linalg.inv(T_local_to_world)
    
    # Convert the desired position to homogeneous coordinates [x, y, z, 1]
    desired_position_homogeneous = np.array([x_des_global, y_des_global, self.actual['z_base'], 1])
    actual_position_homogeneous = np.array([self.actual['x_base'], self.actual['y_base'], self.actual['z_base'], 1])
    
    # Apply the transformation to get the local position
    local_position_homogeneous = T_world_to_local @ desired_position_homogeneous
    actual_local_position_homogeneous = T_world_to_local @ actual_position_homogeneous
    local_velocity_vector = (R_local_to_world.T) @ np.array([[x_dot_des_global],[y_dot_des_global]])
    actual_local_velocity_vector = (R_local_to_world.T) @ np.array([[self.actual['x_base_dot']],[self.actual['y_base_dot']]])
    
    # Extract the local position from the homogeneous coordinates
    local_position = local_position_homogeneous[:3]
    actual_local_position = actual_local_position_homogeneous[:3]
    
    self.desired['x_base_local'] = local_position[0]
    self.desired['y_base_local'] = local_position[1]
    self.actual['x_base_local'] = actual_local_position[0]
    self.actual['y_base_local'] = actual_local_position[1]
    self.desired['x_base'] = x_des_global
    self.desired['y_base'] = y_des_global
    
    self.desired['x_base_dot_local'] = local_velocity_vector[0]
    self.desired['y_base_dot_local'] = local_velocity_vector[1]
    self.actual['x_base_dot_local'] = actual_local_velocity_vector[0]
    self.actual['y_base_dot_local'] = actual_local_velocity_vector[1]
    self.desired['x_base_dot'] = x_dot_des_global
    self.desired['y_base_dot'] = y_dot_des_global
    
  def get_desired_hipy_state(self, xf_des, xf_dot_des):
    z_pos = self.params['leg_length'] 
    z_vel = 0 
    if xf_des > 0:
        alpha = math.atan(abs(xf_des)/z_pos)
        alpha_dot = z_pos/(z_pos**2 + xf_des**2)*xf_dot_des + (xf_des/(xf_des**2 + z_pos**2))*z_vel
    elif xf_des < 0:
        alpha = -math.atan(abs(xf_des)/z_pos)
        alpha_dot = z_pos/(z_pos**2 + xf_des**2)*xf_dot_des + (xf_des/(xf_des**2 + z_pos**2))*z_vel
    else:
        alpha = 0
        alpha_dot = 0
    hip_des = -alpha - self.actual['pitch_base']
    hip_dot_des = -alpha_dot - self.actual['pitch_base_dot']
    
    self.desired['hipy'] = hip_des
    self.desired['hipy_dot'] = hip_dot_des
    return hip_des, hip_dot_des

  def get_desired_hipx_state(self, yf_des, yf_dot_des):
    z_pos = self.params['leg_length']
    z_vel = 0
    if yf_des > 0:
        alpha = math.atan(abs(yf_des)/z_pos)
        alpha_dot = z_pos/(z_pos**2 + yf_des**2)*yf_dot_des + (yf_des/(yf_des**2 + z_pos**2))*z_vel
    elif yf_des < 0:
        alpha = -math.atan(abs(yf_des)/z_pos)
        alpha_dot = z_pos/(z_pos**2 + yf_des**2)*yf_dot_des + (yf_des/(yf_des**2 + z_pos**2))*z_vel
    else:
        alpha = 0
        alpha_dot = 0
    hip_des = alpha - self.actual['roll_base']
    hip_dot_des = alpha_dot - self.actual['roll_base_dot']
    
    self.desired['hipx'] = hip_des
    self.desired['hipx_dot'] = hip_dot_des
    return hip_des, hip_dot_des

  def update_state_machine(self, t):
    phase = (t - self.hop_history['time_of_switch'])/0.1
    phase = min(max(phase, 0), 1) 
    
    if self.state == State.FLIGHT and self.sensor['grf'] > 0 and self.actual['z_base_dot'] < 0 and phase == 1:
      
      self.params['leg_color_phase'] = (self.params['leg_color_phase'] + 0.1) % 1
      r = max(0, min(1, 2 * (1 - abs(3 * self.params['leg_color_phase'] - 1))))
      g = max(0, min(1, 2 * (1 - abs(3 * self.params['leg_color_phase'] - 2))))
      b = max(0, min(1, 2 * (1 - abs(3 * self.params['leg_color_phase'] - 3))))
      self.params['leg_color'] = [r, g, b, 1]
            
      self.state = State.STANCE
      self.hop_history['time_of_switch'] = t
      # print("Switching to Stance")
      
      # Store the last hop height:
      if self.params['first_hop_flag'] == False:
        self.hop_history['last_hop_height'] = self.hop_history['temp_max_height']
        self.hop_history['temp_max_height'] = 0
        # print("Last Hop Height: ", self.hop_history['last_hop_height'])
        
        self.hop_history['adapted_hop_torque'] = self.hop_history['adapted_hop_torque'] + 100*(self.params['desired_hop_height'] - self.hop_history['last_hop_height'])
        # print("Adapted Hop Torque: ", self.hop_history['adapted_hop_torque'])
        # print("Base Hop Torque: ", self.params['base_hop_torque'])
        
    if self.state == State.STANCE and self.sensor['grf'] == 0 and self.actual['z_base_dot'] > 0 and phase == 1:
      
      #Update switching time
      if self.params['first_hop_flag'] == False:
        # self.hop_history['T_stance'] = self.hop_history['time_of_switch'] - t
        self.hop_history['T_stance'] = self.hop_history['T_stance']
      else: 
        self.params['first_hop_flag'] = False
        print("FIRST HOP FINISHED")
        
      self.state = State.FLIGHT
      self.hop_history['time_of_switch'] = t
      # print("Switching to Flight")
      
      
      
    def update_hop_torque(self):
      
      
      # Update hop torque based on the last hop height
      # if self.last_hop_height > self.params['desired_hop_height']:
      #   self.hop_history['adapted_hop_torque'] = self.params['base_hop_torque'] - 10
      # else:
      #   self.hop_history['adapted_hop_torque'] = self.params['base_hop_torque'] + 10
      pass