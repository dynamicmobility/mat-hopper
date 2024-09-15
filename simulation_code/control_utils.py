import numpy as np
import math

def get_torques(hopper):
    hipy_torque = -10*(hopper.actual['hipy'] - hopper.desired['hipy']) - 0.1*(hopper.actual['hipy_dot'] - hopper.desired['hipy_dot']) 
    hipx_torque = -10*(hopper.actual['hipx'] - hopper.desired['hipx']) - 0.1*(hopper.actual['hipx_dot'] - hopper.desired['hipx_dot']) 
    
    # print("Hip X Torque:", hipx_torque)
    # print("Hip Y Torque:", hipy_torque)
    
    return hipy_torque, hipx_torque

def x_base_position_control(hopper):
    xvel_max = hopper.params['max_vel']
    xvel_des_unfiltered = 0.2*hopper.desired['x_base_dot_local'] -0.5*(hopper.actual['x_base_local'] - hopper.desired['x_base_local'])
    xvel_des = max(min(xvel_des_unfiltered,xvel_max),-xvel_max)
    xfd = x_velocity_control(hopper, xvel_des)
    return xfd

def y_base_position_control(hopper):
    yvel_max = hopper.params['max_vel']
    yvel_des_unfiltered = 0.2*hopper.desired['y_base_dot_local'] -0.5*(hopper.actual['y_base_local'] - hopper.desired['y_base_local'])
    yvel_des = max(min(yvel_des_unfiltered,yvel_max),-yvel_max)
    yfd = y_velocity_control(hopper, yvel_des)
    return yfd

def x_velocity_control(hopper, xvel_des):
    T_stance = hopper.hop_history['T_stance']
    x_foot_des = (hopper.actual['x_base_dot_local']*T_stance)/2 + 0.05*(hopper.actual['x_base_dot_local'] - xvel_des)
    return x_foot_des

def y_velocity_control(hopper, yvel_des):
    T_stance = hopper.hop_history['T_stance']
    y_foot_des = (hopper.actual['y_base_dot_local']*T_stance)/2 + 0.05*(hopper.actual['y_base_dot_local'] - yvel_des)
    return y_foot_des