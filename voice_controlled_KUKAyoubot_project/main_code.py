# -*- coding: utf-8 -*-
"""
Created on Sun Jan 10 23:15:14 2021

@author: Nikhil
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
import speech_recognition as sr
import pyttsx3
import datetime 
import os

engine = pyttsx3.init('sapi5') 
voices = engine.getProperty('voices') 
engine.setProperty('voice', voices[0].id)


def speak(audio): 
	engine.say(audio) 
	engine.runAndWait() 

def wishMe(): 
    hour = int(datetime.datetime.now().hour) 
    if hour>= 0 and hour<12: 
        speak("Good Morning mister nikhil !") 
   
    elif hour>= 12 and hour<18: 
        speak("Good Afternoon mister nikhil !")    
   
    else: 
        speak("Good Evening mister nikhil !")   
   
    assname =("my name is Jarvis") 
    #speak("I am your You bot sir") 
    speak(assname)
    speak("what should i do")


def takeCommand(): 
      
    r = sr.Recognizer() 
      
    with sr.Microphone() as source: 
          
        print("Listening...") 
        r.pause_threshold = 1
        audio = r.listen(source) 
   
    try: 
        print("Recognizing...")     
        query = r.recognize_google(audio, language ='en-in') 
        print(f"User said: {query}\n") 
   
    except Exception as e: 
        print(e)     
        print("Unable to Recognizing your voice.")   
        return "None"
      
    return query 


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

    def ComputeTrajectory(self, deseired):
        '''
        Compute Full trajectory and returns the trajectory
        Return:
            Trajectory : [[[np.array([.Tse..]), np.array([..Tse.])], 0]]  (first part is the trajectory and the second the state of the gripper)
        '''

        trajectories = list()
        # Tse_initial to Tse_init_standoff
        Tse_init_standoff = np.dot(desired,self.Tce_standoff)
        trajectories.append([self.traj_gen.Generate_segment_trajectory(self.Tse_init, Tse_init_standoff, self.time_scale, self.traj_type),self.gripper_open])

        return trajectories
    
    
if __name__ == '__main__': 
    clear = lambda: os.system('cls') 
    
    folder_name = 'newTask'
    
    #present_state = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
    
    bot_params_filename = 'config/bot_params.yaml'
    traj_params_filename = 'config/trajectory_params.yaml'
    conf_csv_filename = 'config.csv'
    traj_csv_filename = 'trajectory.csv'
    plot_fileneame='plot.png'
    error_csv_filename='error.csv'

    # Reading params from the params file
    with open(bot_params_filename) as file:  
        bot_params = yaml.load(file)

    # Reading params from the params file
    with open(traj_params_filename) as file:  
        traj_params = yaml.load(file)

    full_configs = list()
    error_list = [[],[]]
    kuka_bot = Robot(bot_params, traj_params)
    config = bot_params["initial_config"]
    # Adding initial config
    full_configs.append([round(conf,4) for conf in config["chasis"]]+ [round(conf,4) for conf in config["arm"]] + [round(conf,4) for conf in config["wheel"]] + [config["gripp_state"]])
    time = 0.0
    timestep = traj_params["timestep"]
    
    # This Function will clean any 
    # command before execution of this python file
    wishMe()      
    #t = {'one':1,'two':2,'three':3,'four':4,'five':5,'six':6,'seven':7,'eight':8,'nine':9,'ten':10}
    while True:
        query = takeCommand().lower() 
          
        # All the commands said by user will be  
        # stored here in 'query' and will be 
        # converted to lower case for easily  
        # recognition of command 
        if 'activate voice control' in query:
            speak("activating voice control")
            speak("mention the goal X coordinate")
            query = takeCommand().lower() 
            # a = 1, for testing
            a = int(query)
            speak("mention the goal Y coordinate")
            query = takeCommand().lower() 
            # b = 1, for testing
            b = int(query)
            speak("mention the goal Z coordinate")
            query = takeCommand().lower() 
            # c = 3, for testing
            c = int(query)/100  #c is the z coordinate of goal config wrt to space frame, it should be less than 0.5 orelse the robot might get broken down, thats why we have scaled it by dividing it wih 100 
            #c = 0
            desired = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
            desired[0,3] = desired[0,3] + a 
            desired[1,3] = desired[1,3] + b
            desired[2,3] = desired[2,3] + c
            trajs = kuka_bot.ComputeTrajectory(desired)
            #present_state = desired_state
            for traj,gripp_state in trajs:
                # Traverse through each configuration in the trajectory
                for conf_no in range(len(traj)-1):
                    Tse_des = traj[conf_no] # Desired configuration
                    Tse_des_next = traj[conf_no + 1] # Desired configuration after timestep
                    if type(config) == type(dict()):
                        confy = config
                    else:
                        confy = {}
                        confy["arm"] = config[3:8]
                        confy["chasis"] = config[0:3]
                        confy["wheel"] = config[8:12]
                        confy["gripper state"] = config[12]
                    # Computing required wheel and joint velocity to achieve the desired configuration
                    end_eff_twist, wheel_vel, joint_vel = kuka_bot.feedback_controller.FeedbackControl(confy, Tse_des, Tse_des_next)
                    error_list[0].append(time)
                    error_list[1].append(end_eff_twist)            
                    controls = {'arm' : list(joint_vel), 'wheel' : list(wheel_vel)}
                    new_state = kuka_bot.odometry.NextState(confy, controls, timestep) # Compute next configuration after timestep
                    full_configs.append([round(conf,4) for conf in new_state["chasis"]]+ [round(conf,4) for conf in new_state["arm"]] + [round(conf,4) for conf in new_state["wheel"]] + [gripp_state])
                    #new_state = kuka_bot.odometry.NextState(config, controls, timestep)
                    config = new_state# Updating configuration
                    time = time + timestep

                    # Generate config file
                    print ("Generating config csv file")
                    with open(conf_csv_filename, "w") as csv_file:
                        writer = csv.writer(csv_file, delimiter=',')
                        for config in full_configs:
                            writer.writerow(config)
            
        
        elif "sleep" in query:
            speak("okay im sleeping")
            speak("have a nice day sir")
            speak("it is an honour to be built by you")
            break
        else:
            speak("unable to recognize your voice sir")


    
   
    


