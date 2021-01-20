"""
Created on Mon 20, 2020

"""

import numpy as np
import modern_robotics as mr
import yaml
import csv
from math import cos,sin, acos, atan2, copysign, fabs


class Odometry:

    def __init__(self,bot_params):

        # Vb = F*Δθ
        l = bot_params["chasis"]["l"]
        w = bot_params["chasis"]["w"]
        self.rad = bot_params["chasis"]["wheel_rad"]
        self.height = bot_params["chasis"]["height"]

        row_val = 1.0/(l+w)
        
        F = (self.rad/4) * np.array([[-row_val, row_val, row_val, -row_val],[1,1,1,1],[-1,1,-1,1]])
        self.F6 = np.array([[0,0,0,0],[0,0,0,0],F[0],F[1],F[2],[0,0,0,0]]) # this maps change in wheel angle to body twist

        self.limit_arm = bot_params["velocity_limits"]["arm"]
        self.limit_wheel = bot_params["velocity_limits"]["wheel"]

    def NextState(self, config, controls, timestep):
        ''' 
        Input  : Chasis configuration, arm and wheel controls, timestep and dimensions of the robot
            config : { chasis: [phi,x,y], arm: [0,0,0,0,0], wheel : [0,0,0,0] }
            controls : {arm : [joint_1_speed, 2,3,4], wheel : [wheel 1 speed, 2,3,4]}
            timestep : delta time 
        Output : New configuration after the controls for timestep
            new_config : { chasis: [phi,x,y], arm: [0,0,0,0,0], wheel : [0,0,0,0] }
        '''

        new_config = {}
        new_config["chasis"] = [0,0,0]
        new_config["arm"] = [0,0,0,0,0]
        new_config["wheel"] = [0,0,0,0]

        # New wheel configuration
        del_theta_wheel = [0]*len(new_config["wheel"]) 
        del_theta_joint = [0]*len(new_config["arm"])
        for wheel_no in range(len(new_config["wheel"])):
            # Checking if wheel speed is off limits
            if fabs(controls["wheel"][wheel_no]) > fabs(self.limit_wheel[wheel_no]):
                print ("Wheel velocity exceeded for wheel ",wheel_no + 1, controls["wheel"][wheel_no])
                controls["wheel"][wheel_no] = copysign(self.limit_wheel[wheel_no],controls["wheel"][wheel_no])
                

            del_theta_wheel[wheel_no] = controls["wheel"][wheel_no] * timestep # delta theta to be multiplied with F6 to find Vb6
            new_config["wheel"][wheel_no] = config["wheel"][wheel_no] + del_theta_wheel[wheel_no]
        
        # New joint angles
        for joint_no in range(len(new_config["arm"])):
            # Checking if arm joint speed is off limits
            if fabs(controls["arm"][joint_no]) > fabs(self.limit_arm[joint_no]):
                print ("Joint velocity exceeded for Joint ",joint_no + 1, controls["arm"][joint_no])
                controls["arm"][joint_no] = copysign(self.limit_arm[joint_no],controls["arm"][joint_no])
                

            del_theta_joint[joint_no] = controls["arm"][joint_no] * timestep
            new_config["arm"][joint_no] = config["arm"][joint_no] + del_theta_joint[joint_no]
        
        phi = config["chasis"][0]
        x = config["chasis"][1]
        y = config["chasis"][2]

        Tsb = np.array([[cos(phi),sin(phi),0,0],[-sin(phi),cos(phi),0,0],[0,0,1,0],[x,y,self.height,1]]).T

        # Finding skew symmetric matrix and integrate to find the delta transformation matrix
        Vb6 = self.F6.dot(del_theta_wheel) # Multiplied with timestep as Vb6 is the twist for unit time
        se3mat = mr.VecTose3(Vb6)
        Tbk_b1k = mr.MatrixExp6(se3mat)

        # New Position of the bot wrt space frame
        Tsb1k = Tsb.dot(Tbk_b1k)
        # theta = acos(Tsb1k[0,0]) # 0th element is cos(phi)..so inverse gives phi
        theta = atan2(Tsb1k[1,0],Tsb1k[0,0]) # phi in range of -pi to pi

        new_config["chasis"][0] = theta
        new_config["chasis"][1] =  Tsb1k[0,-1]
        new_config["chasis"][2] =  Tsb1k[1,-1]

        # print config, controls, limits

        return new_config # chasis, arm, wheel


if __name__ == "__main__":

    params_filename = 'config/test_odom.yaml'
    bot_params_filename = 'config/bot_params.yaml'
    csv_filename = '../results/Debug/test_odom.csv'

    # Reading params from the params file
    with open(params_filename) as file:  
        params = yaml.load(file)
    
    # Reading params from the params file
    with open(bot_params_filename) as file:  
        bot_params = yaml.load(file)
    
    # new_config = NextState(params["config"], params["controls"], params["limits"], params["timestep"],bot_params)
    iterations = 1000

    test_odom = Odometry(bot_params)

    configs = list()
    for iter in range(iterations):
        new_config = test_odom.NextState(params["config"], params["controls"], params["timestep"])
        params["config"] = new_config
        configs.append(new_config["chasis"] + new_config["arm"] + new_config["wheel"] +[0]) # chasis, arm, wheel, gripper state


    with open(csv_filename, "w") as csv_file:
            writer = csv.writer(csv_file, delimiter=',')
            for config in configs:
                writer.writerow(config)
    

    print ("\nControls : ",params["controls"])
    # print (params["controls"])

    print ("\nTimestep(s) : ",params["timestep"]*iterations)

    for key,val in new_config.items():
        new_config[key] = [round(ele,3) for ele in val]

    print ("\nNew configuration : \n",new_config)
    print ("\n")
    
    
