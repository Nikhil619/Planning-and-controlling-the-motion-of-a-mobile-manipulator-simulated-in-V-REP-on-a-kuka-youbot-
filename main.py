"""
Created on Mon 20, 2020

@author : Fayyaz Pocker Chemban
"""

import numpy as np
import modern_robotics as mr
import yaml
from trajectory import TrajectoryGenerator
from simulate import Odometry
from feedback_controller import FeedbackController
from math import cos, sin
import csv
import matplotlib.pyplot as plt

class Robot:

    def __init__(self,bot_params, traj_params):


        # Static Configurations 
        self.Tsc_init = np.array(traj_params["Tsc_initial"])
        self.Tce_standoff = np.array(traj_params["Tce_standoff"])
        self.Tce_grasp = np.array(traj_params["Tce_grasp"])
        self.Tsc_final = np.array(traj_params["Tsc_final"])
        self.time_scale = traj_params["time_scaling"]
        self.traj_type = traj_params["trajectory_type"]

        # Parameters of the bot
        init_phi = bot_params["initial_config"]["chasis"][0]
        init_x = bot_params["initial_config"]["chasis"][1]
        init_y = bot_params["initial_config"]["chasis"][2]
        height = bot_params["chasis"]["height"]

        # Tbo = np.array(bot_params["arm"]["Tbo"])  # chasis frame wrt arm base
        # Moe = np.array(bot_params["arm"]["Moe"]) # Arm at home config (all joint angles zero).. end effector {e} wrt arm base {o}
        # Blist = np.array(bot_params["arm"]["Blist"]) # Blist of arm
        # thetalist = np.array(bot_params["initial_config"]["arm"])
        # Tsb_init = np.array([[cos(init_phi),-sin(init_phi),0,init_x],[sin(init_phi),cos(init_phi),0,init_y],[0,0,1,height],[0,0,0,1]]) # chasis base wrt space frame
        # Toe_init = mr.FKinBody(Moe, Blist, thetalist) # base of the arm wrt to end effector
        # self.Tse_init = np.dot(np.dot(Tsb_init,Tbo),Toe_init) # Initial end effector wrt space frame
        self.Tse_init = traj_params["Tse_initial"]

        self.gripper_open = 0
        self.gripper_closed = 1

        # Trajectory Generator 
        self.traj_gen = TrajectoryGenerator(traj_params["max_lin_vel"], traj_params["max_ang_vel"], traj_params["k"])

        # Feedback controller
        self.feedback_controller = FeedbackController(traj_params["Kp"], traj_params["Ki"], traj_params["timestep"], bot_params)

        # Simulator to measure odometry
        self.odometry = Odometry(bot_params)

    def ComputeTrajectory(self):
        '''
        Compute Full trajectory and returns the trajectory
        Return:
            Trajectory : [[[np.array([.Tse..]), np.array([..Tse.])], 0]]  (first part is the trajectory and the second the state of the gripper)
        '''

        trajectories = list()
        # Tse_initial to Tse_init_standoff
        Tse_init_standoff = np.dot(self.Tsc_init,self.Tce_standoff)
        trajectories.append([self.traj_gen.Generate_segment_trajectory(self.Tse_init, Tse_init_standoff, self.time_scale, self.traj_type),self.gripper_open])

        # Tse_init_standoff to Tse_init_grasp
        Tse_init_grasp = np.dot(self.Tsc_init,self.Tce_grasp)
        trajectories.append([self.traj_gen.Generate_segment_trajectory(Tse_init_standoff, Tse_init_grasp, self.time_scale, self.traj_type),self.gripper_open])

        # Close gripper
        trajectories.append([self.traj_gen.GripperChange(Tse_init_grasp),self.gripper_closed])

        #Tse_init_grasp to Tse_init_standoff
        trajectories.append([self.traj_gen.Generate_segment_trajectory(Tse_init_grasp, Tse_init_standoff, self.time_scale, self.traj_type),self.gripper_closed])

        # Tse_init_grasp to Tse_final_standoff
        Tse_final_standoff = np.dot(self.Tsc_final,self.Tce_standoff)
        trajectories.append([self.traj_gen.Generate_segment_trajectory(Tse_init_standoff, Tse_final_standoff, self.time_scale, self.traj_type), self.gripper_closed])

        # Tse_final_standoff to Tse_final_grasp
        Tse_final_grasp = np.dot(self.Tsc_final,self.Tce_grasp)
        trajectories.append([self.traj_gen.Generate_segment_trajectory(Tse_final_standoff, Tse_final_grasp, self.time_scale, self.traj_type), self.gripper_closed])

        # Open Gripper
        trajectories.append([self.traj_gen.GripperChange(Tse_final_grasp), self.gripper_open])

        return trajectories


if __name__ == "__main__":

    folder_name = 'newTask'
    
    bot_params_filename = 'config/bot_params.yaml'
    traj_params_filename = 'config/trajectory_params.yaml'
    conf_csv_filename = '../results/' + folder_name + '/config.csv'
    traj_csv_filename = '../results/' + folder_name + '/trajectory.csv'
    plot_fileneame='../results/' + folder_name + '/plot.png'
    error_csv_filename='../results/' + folder_name + '/error.csv'

    # Reading params from the params file
    with open(bot_params_filename) as file:  
        bot_params = yaml.load(file)

    # Reading params from the params file
    with open(traj_params_filename) as file:  
        traj_params = yaml.load(file)

    full_configs = list()
    error_list = [[],[]]# Stores Xerr wrt to time.. [[Xerr1, Xerr2,..],[time1, time2,..]]
    
    kuka_bot = Robot(bot_params, traj_params)
    trajs = kuka_bot.ComputeTrajectory() # Compute trajectory of end effector wrt space frame
    
    config = bot_params["initial_config"]
    # Adding initial config
    full_configs.append([round(conf,4) for conf in config["chasis"]]+ [round(conf,4) for conf in config["arm"]] + [round(conf,4) for conf in config["wheel"]] + [config["gripp_state"]])
    time = 0.0
    timestep = traj_params["timestep"]

    # Compute joint and wheel velocities for desired trajectory
    for traj,gripp_state in trajs:
        # Traverse through each configuration in the trajectory
        for conf_no in range(len(traj)-1):
            Tse_des = traj[conf_no] # Desired configuration
            Tse_des_next = traj[conf_no + 1] # Desired configuration after timestep
            
            # Computing required wheel and joint velocity to achieve the desired configuration
            end_eff_twist, wheel_vel, joint_vel = kuka_bot.feedback_controller.FeedbackControl(config, Tse_des, Tse_des_next)
            error_list[0].append(time)
            error_list[1].append(end_eff_twist)            
            controls = {'arm' : list(joint_vel), 'wheel' : list(wheel_vel)}
            new_state = kuka_bot.odometry.NextState(config, controls, timestep) # Compute next configuration after timestep
            full_configs.append([round(conf,4) for conf in new_state["chasis"]]+ [round(conf,4) for conf in new_state["arm"]] + [round(conf,4) for conf in new_state["wheel"]] + [gripp_state])
            config = new_state # Updating configuration
            time = time + timestep

    # Generate config file
    print ("Generating config csv file")
    with open(conf_csv_filename, "w") as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        for config in full_configs:
            writer.writerow(config)
    
    # Generate csv file of trajectory
    traj_csv = list()
    for traj_seg in trajs:
        for traj_conf in traj_seg[0]:
            new_traj = list(np.concatenate(traj_conf[:-1,:-1],axis=0)) + list(np.concatenate(traj_conf[:-1,-1:],axis=0)) # Converting into csv compatible
            new_traj.append(traj_seg[1]) # open gripper status
            traj_csv.append(new_traj)
    
    # Write csv file
    print ("Generating trajectory csv file")
    with open(traj_csv_filename, "w") as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        for traj_seg in traj_csv:
            writer.writerow(traj_seg)
    
    plt.xlabel('Time')
    plt.ylabel('Error twist')

    plt.plot(error_list[0],error_list[1])
    print ("Generating plot image")
    plt.savefig(plot_fileneame) # Save plot of error

    error_csv = []
    for error_no in range(len(error_list[0])):
        new_error = list()
        new_error = [error_list[0][error_no]] + list(error_list[1][error_no])
        error_csv.append(new_error)
    
    print ("Generating error csv file")
    with open(error_csv_filename, "w") as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        for error in error_csv:
            writer.writerow(error)



    # plt.show()
    # print (error_list[0])

    # print (kuka_bot.trajectories)