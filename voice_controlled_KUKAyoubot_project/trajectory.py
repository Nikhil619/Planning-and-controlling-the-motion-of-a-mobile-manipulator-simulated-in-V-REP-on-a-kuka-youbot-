import numpy as np
import modern_robotics as mr
import csv
import yaml
import math

class TrajectoryGenerator:

    def __init__(self, max_lin_vel, max_ang_vel, k):
        '''
        Initialize the input parameters
        '''

        self.k = k
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel

        self.gripper_open = 0
        self.gripper_closed = 1
    

    def Generate_segment_trajectory(self,T_start, T_end, time_scale, traj_type):
        '''
        Generate segment of trajectory of the end effector
        Input : 
            T_start : Starting configuration
            T_end : Goal configuration
            time_scale : Time scaling of this segment
            traj_type : Screw or Cartesian type
        
        Return:
            trajs : Trajectory of the segment in [np.array([Tse..]), np.array([next_Tse])]
        '''

        # Duration and number of configuration of the trajectory is computed based on the maximum linear and angular velocity
        T_dist = np.dot(mr.TransInv(T_start),T_end)
        dist = math.hypot(T_dist[0,-1], T_dist[1,-1])
        ang = max(self.euler_angles_from_rotation_matrix(T_dist[:-1,:-1]))

        duration = max(dist/self.max_lin_vel, ang/self.max_ang_vel) # Duration from rest to rest
        no_conf = int((duration * self.k)/0.01) # Number of configuration between the start and goal

        if (traj_type == "Screw"):
            trajs = mr.ScrewTrajectory(T_start, T_end, duration, no_conf, time_scale)
        elif(traj_type == "Cartesian"):
            trajs = mr.CartesianTrajectory(T_start, T_end, duration, no_conf, time_scale)
        
        return trajs

    def GripperChange(self,Tse_current):
        '''
        Change the status of the gripper. There must be 63 consecutive line to ensure proper opening of the gripper
        Input:
            Tse_current : Current configuration
        
        return:
            Trajectory with 1 as the grip state
        '''

        trajectory = list()
        for i in range(63):
            trajectory.append(Tse_current)
        
        return trajectory

    def euler_angles_from_rotation_matrix(self,R):
        '''
        Converts rotation matrix to euler angles
        Input :
            R : Rotation matrix
        
        Return :
            alpha, beta and gamma
        '''

        def isclose(x, y, rtol=1.e-5, atol=1.e-8):
            return abs(x-y) <= atol + rtol * abs(y)

        phi = 0.0
        if isclose(R[2,0],-1.0):
            theta = math.pi/2.0
            psi = math.atan2(R[0,1],R[0,2])
        elif isclose(R[2,0],1.0):
            theta = -math.pi/2.0
            psi = math.atan2(-R[0,1],-R[0,2])
        else:
            theta = -math.asin(R[2,0])
            cos_theta = math.cos(theta)
            psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
            phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
        return psi, theta, phi

    def convert_traj_csv_format(self,trajs, gripp_state):
        '''
        Converts the trajectory to csv compatible format
        Return:
            trajectory : [r11,r12,r13,r21,..., px,py,pz,gripper_state]
        '''
        trajectory = list()
        for traj in trajs:
            new_traj = list(np.concatenate(traj[:-1,:-1],axis=0)) + list(np.concatenate(traj[:-1,-1:],axis=0)) # Converting into csv compatible
            new_traj.append(gripp_state) # open gripper status
            trajectory.append(new_traj)
        
        return trajectory



if __name__ == "__main__":

    params_filename = 'config/test_trajectory_params.yaml'
    csv_filename = '../results/Debug/test_trajectory.csv'

    # Reading params from the params file
    with open(params_filename) as file:  
        params = yaml.load(file)
    
    # List to save the full trajectory
    full_trajectory = list()
    

    Tse_init = np.array(params["Tse_initial"])
    Tsc_init = np.array(params["Tsc_initial"])
    Tce_standoff = np.array(params["Tce_standoff"])
    Tce_grasp = np.array(params["Tce_grasp"])
    Tsc_final = np.array(params["Tsc_final"])

    k = params["k"]
    max_lin_vel = params["max_lin_vel"]
    max_ang_vel = params["max_ang_vel"]
    time_scale = params["time_scaling"]
    traj_type = params["trajectory_type"]

    gripper_open = 0
    gripper_closed = 1

    traj_gen = TrajectoryGenerator(max_lin_vel, max_ang_vel, k)

    # Tse_initial to Tse_init_standoff
    Tse_init_standoff = np.dot(Tsc_init,Tce_standoff)
    full_trajectory = traj_gen.convert_traj_csv_format(traj_gen.Generate_segment_trajectory(Tse_init, Tse_init_standoff, time_scale, traj_type), gripper_open)

    # Tse_init_standoff to Tse_init_grasp
    Tse_init_grasp = np.dot(Tsc_init,Tce_grasp)
    full_trajectory = full_trajectory + traj_gen.convert_traj_csv_format(traj_gen.Generate_segment_trajectory(Tse_init_standoff, Tse_init_grasp, time_scale, traj_type),gripper_open)

    # Close gripper
    full_trajectory = full_trajectory + traj_gen.convert_traj_csv_format(traj_gen.GripperChange(Tse_init_grasp), gripper_closed)

    # Tse_init_grasp to Tse_final_standoff
    Tse_final_standoff = np.dot(Tsc_final,Tce_standoff)
    full_trajectory = full_trajectory + traj_gen.convert_traj_csv_format(traj_gen.Generate_segment_trajectory(Tse_init_grasp, Tse_final_standoff, time_scale, traj_type), gripper_closed)

    # Tse_final_standoff to Tse_final_grasp
    Tse_final_grasp = np.dot(Tsc_final,Tce_grasp)
    full_trajectory = full_trajectory + traj_gen.convert_traj_csv_format(traj_gen.Generate_segment_trajectory(Tse_final_standoff, Tse_final_grasp, time_scale, traj_type), gripper_closed)

    # Open Gripper
    full_trajectory = full_trajectory + traj_gen.convert_traj_csv_format(traj_gen.GripperChange(Tse_final_grasp), gripper_open)

    # Write csv file
    with open(csv_filename, "w") as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        for traj_seg in full_trajectory:
            writer.writerow(traj_seg)
