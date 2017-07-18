#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 17 14:26:20 2017

@author: paul
"""

import time
#import inspect
#import shutil
#import os
import sys
#import scipy.io as io
#import matplotlib.pyplot as pyplot
import numpy
#import warnings
#import threading
#import Queue
import multiprocessing as mp

#from library import HTMLlog
from library import Utilities
from library import Vicon
from library import Robot
from library import PathPlanning

WARN_TO = 3



class human_controller():
    #Class for planning and control of human robot, including responses to events in the environment
    def __init__(self, tracker, robot_q, human_q, session_path, settings, name):
        self.tracker = tracker
        self.log_file = session_path
        self.robot_q = robot_q #Queue.Queue created in main thread that allows robot to pass messages
        self.human_q = human_q #Queue.Queue created in main thread that allows robot to get messages from human(s)
        self.settings = settings
        
        self.Experiment_Logger = Utilities.Logger(name + ' robot')
        self.human_graph = PathPlanning.Planner(self.tracker)
        self.name = name
        self.travelled_path = []

        if name + '_obstacles' in self.settings:#if a starting set of objects the robot knows about self_obstacles ['TARGET_A','TARGET_B']
            self.human_graph.set_obstacles(self.settings[name + '_obstacles'])

        #if 'dangers' in self.settings:#if a starting set of dangers the robot knows about
            #currently dangers just exist in the CE
            #self.CE.set_dangers(self.settings['dangers'])#at experiment start the robot assumes humans are ignorant of all dangers so only adds them to its own map

        #all execution of the human control happens in a seperate process
        self.plan_process = mp.Process(target=self.plan_Execute)
        
        
    def plan_Execute(self):
        responce_ctr = 0
        already_responded = False  
        #create connection to robot
        if self.tracker:
            robot = Robot.Robot(self.settings[self.name+'_ip'], self.name, self.tracker)
            #set up robot and move to starting location
            robot.speed_factor = self.settings[self.name + '_speed_factor']
            robot.update_background_checks(0, False)
            robot.update_background_checks(2, False)
            robot.go_to_position(self.settings[self.name + '_start'][0], self.settings[self.name + '_start'][1], blocking=True)
            robot.go_to_orientation(self.settings[self.name + '_start_angle'], blocking=True)
            #message exchange to sync experiment start
        self.human_q.put(self.name + ' ready to start')#signal to the supervisor human is in position
        start_msg = self.robot_q.get()#block on the signal sent via the robot q all are ready to start the experiment
        self.Experiment_Logger.write(start_msg)
        
        for sim_steps in range(1000):
            start=time.time()
            human_goal = self.settings[self.name + '_goal']
            
            if 'DEBUG_position_human' in self.settings:
                current_position_human = numpy.array(self.settings['DEBUG_position_human'])
            else:
                current_position_human = self.tracker.get_position(self.name)[0:2]

            self.travelled_path.append(current_position_human)
            
            if 'DEBUG_position_human' in self.settings:
                current_position_robot = numpy.array(self.settings['DEBUG_robot_location'])
            else:
                current_position_robot = self.tracker.get_position('ROBOT')[0:2]

            inter_robot_distance = numpy.sum((current_position_robot - current_position_human)**2)**0.5
            #inter_robot_distance = 1 #debug 
            if inter_robot_distance < 0.5:
                robot.stop_robot()
                robot.clean_up()
                break
            
            #response timeout to allow both robots to grab only 1 warning message from the queue while the other grabs a 2nd
            if already_responded:
                if responce_ctr < WARN_TO:
                    responce_ctr = responce_ctr + 1
                else:
                    responce_ctr = 0
                    already_responded = False
                
            #if robot has warned or pointed and human has not responded then respond if close enough at time of warning
            if not self.robot_q.empty() and not already_responded:
                warning_msg = self.robot_q.get()
                warning_type = warning_msg['warning_type']
                ack_msg = {'name':self.name, 'type': warning_type}#could add location heard
                robot.speak_text('warning heard')
                already_responded = True
                responce_ctr = 0
                #if robot warns of danger assume goal is dangerous and add it as an obstacle 
                if warning_type == 'warn' and inter_robot_distance < self.settings['hearing_dist']:#hearing_dist can be set to a low number to make the human never ack if wanted
                    self.human_graph.add_obstacles(warning_msg['position'])#add warned about danger to graph
                    self.human_q.put(ack_msg)#msg dict contains type of warning ack is for and name of robot
                    self.Experiment_Logger.write('warning heard at ' + str(current_position_human[0]) + ' ' + str(current_position_human[1]))
                #if robot points and it is seen (use inter-robot dist and angle to determine spotting efficacy) then acknowledge and update map if possible
                #if close enough and pointing estimate pointing target, accuracy based on relative angles and distances of robot, should be able to cheat and just distort the actual target pointed to
                elif warning_type == 'point' and inter_robot_distance < self.settings['pointing_dist']:#pointing_dist can be set to a low number to make the human never ack if wanted
                    self.human_graph.add_obstacles(warning_msg['position'])#add pointed to location as an obstacle. TODO add to this so accuracy of point perception is distance and angle dependent
                    self.human_q.put(ack_msg)#msg dict contains type of warning ack is for and name of robot
                    self.Experiment_Logger.write('point seen at ' + str(current_position_human[0]) + ' ' + str(current_position_human[1]))
                elif warning_type == 'debug':
                    self.human_q.put(ack_msg)
                    break
                
            #plan next movement command
            human_motion = self.human_graph.motion_command(current_position_human, human_goal, plot=False)#if plotting should pass the name of the output plot file. Generate the name from the session name and associated directory structure
            self.Experiment_Logger.write('Human Vector Length: ' + str(human_motion['vector_length']))
            next_position_human = human_motion['next_position']
            #human target as designated by the planner, this allows for the goal being identified as a danger
            human_target = human_motion['path']['path'][-1]
            distance_to_goal = (numpy.sum((current_position_human - human_target)**2))**0.5
            self.Experiment_Logger.write('goal ' +  str(human_target[0]) + ' ' + str(human_target[1]) +' dist to goal ' + str(distance_to_goal))
            if distance_to_goal > 0.25: robot.go_to_position(next_position_human[0], next_position_human[1])
            else:
                robot.stop_robot()
                robot.clean_up()
                break

            end = time.time()
            dur = end - start
            try:
                time.sleep(self.settings['update_rate']-dur)
            except:
                pass
            