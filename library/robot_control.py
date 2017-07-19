#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 13:57:06 2017

@author: paul
"""
import time
#import inspect
#import shutil
#import os
#import sys
#import scipy.io as io
#import matplotlib.pyplot as pyplot
#import numpy
#import warnings
#import threading
#import Queue
import multiprocessing as mp
#from library import HTMLlog
from library import Utilities
#from library import Vicon
from library import Robot
#from library import easygui
from library import ethical_engine
#==============================================================================
# TODO: log data of the perceived human world view
# Needs to maintain a model of the human's view of the world for use in planning and assessing consequences, i.e., update the models held by the CE according to 'sensor' data. 
# CE can compare human actions with predicted path for different possible world models to help in assessing plan.
#==============================================================================

class robot_controller():
#class for control of the ethical robot
#It has one process that handles movement commands and plan comparison and 3 processes that invoke CE instances, 1 for each plan type so plans can be simultaneously evaluated
    def __init__(self, tracker, robot_q, human_q, session_path, settings, ip_robot = '192.168.20.224'):
        self.tracker = tracker
        self.log_file = session_path
        #if self.tracker:
        #    self.robot = Robot.Robot(ip_robot, 'ROBOT', self.tracker)
        self.robot_q = robot_q #Queue.Queue created in main thread that allows robot to pass messages
        self.human_q = human_q #list of Queue.Queues created in main thread that allows robot to get messages from human(s)
        #self.ack = threading.Event()
        self.settings = settings
        self.name = 'ROBOT'
        self.ip_robot = ip_robot
        self.Experiment_Logger = Utilities.Logger('ROBOT_control')
        self.Experiment_Logger.write('Generating robot objects')
        
        self.ethical_engine = ethical_engine.ethical_engine(self)
        
 
########create graphs for self and humans in the experiment##############
        
#######################################################################


        self.travelled_path = []

        
        self.human_knowledge = {}#human knowledge is the current beliefs that the robot has of the humans knowledge of the world
        for human in self.settings['humans']:
            self.human_knowledge[human] = []
            if human + '_obstacles' in self.settings:#if a starting set of objects the robot knows about self_obstacles ['TARGET_A','TARGET_B']
                self.human_knowledge[human].append(self.settings[human + '_obstacles'])#initialise list of human knowledge, appended cos knowldge is an array of hypotheses of human knowledge
            else:
                self.human_knowledge[human].append([])            
            #this initial knowledge setting could be modified to instead be estimated from facts about the world, e.g., things in human 'sensor' range    
        self.Experiment_Logger.write('Running script: ' + self.settings['session_name'])
        
        
        self.end_flag = mp.Event()
    
    
    #ACTIONS
    def move_to_target_action(self,robot,action_info):
        #plan and execute robot motion
        plan = action_info['plan']
        robot.speed_factor = plan['speed']
        # Subject to change
        robot_motion = plan['motion_command']
        next_position_robot = robot_motion['next_position']
        self.Experiment_Logger.write('target ' + str(next_position_robot[0])+' '+str(next_position_robot[1]))
        robot.go_to_position(next_position_robot[0], next_position_robot[1])
        robot.speak_text('move to target')#debug
            
    def stop_action(self,robot,action_info):
        robot.stop_robot()
#==============================================================================
#         msgs=rule_info['msgs']
#         self.end_flag.set()#cause CE_processes to terminate on next loop
#         robot.stop_robot()
#         msgs = 0
#         for _ in range(3):
#             self.results_q.task_done()
#==============================================================================

    def vocal_warning_action(self, robot, action_info):
        #if at warn location give warning and signal to human warning given
        consequence_results = action_info['consequence_results']
        robot.speak_text(self.settings['warning_call'], blocking=True)
        position = consequence_results['warn']['humanA_goal']
        self.robot_q.put({'warning_type' : 'warn', 'position':position})
        
    def pointed_warning_action(self, robot, action_info):
        plan = action_info['plan']
        robot.point_direction_world(plan['point_pos'][0], plan['point_pos'][1])
        self.robot_q.put({'warning_type' : 'point','position':plan['point_pos']})
 
#    def vocal_warning_received_rule(self, robot, rule_info):
#==============================================================================
#         danger_locs =[]
#         for danger in self.settings['dangers']:
#                 if isinstance(danger,basestring):
#                     danger_locs.append(self.tracker.get_position(danger)[:2])
#                 else:
#                     danger_locs.append(danger[:])
#                     
#         danger_dists = Utilities.distance2points(current_position_human, danger_locs)
#         danger_dists = numpy.array(danger_dists)
#         min_index = numpy.argmin(danger_dists)
#         min_danger = danger_locs[min_index]
#         update = min_danger
#         self.human_knowledge[msg['name']][hypothesis_selected[msg['name']]].append(update)#append the closest danger to the current hypothesis as the warning was ack'd
#         for plan in ['move','warn','point']:
#             for human in self.settings['humans']:
#                 self.CE[plan].add_obstacles(update, actor = human)
# 
#==============================================================================

#    def pointed_warning_received_rule(self, robot):
#==============================================================================
#         update = plan['point_pos']
#                         
#         self.human_knowledge[msg['name']][hypothesis_selected[msg['name']]].append(update)#append the closest danger to the current hypothesis as the warning was ack'd
#         for plan in ['move','warn','point']:
#             for human in self.settings['humans']:
#                 self.CE[plan].add_obstacles(update, actor = human)
#==============================================================================
 

#    This is the main control loop
    def CE_manager_proc(self):
        #connect to robot and move it to the start
        #self.tracker = 1#debug
        if self.tracker:
            robot = Robot.Robot(self.ip_robot, 'ROBOT', self.tracker)
            robot.speed_factor = self.settings['ROBOT_speed_factor']
            #debug robot.update_background_checks(0, False)
            #debug robot.update_background_checks(2, False)
            #debug robot.go_to_position(self.settings['ROBOT_start'][0], self.settings['ROBOT_start'][1], blocking=True)
            #debug robot.go_to_orientation(self.settings['ROBOT_start_angle'], blocking=True)
            #robot moving blocks until in position, then exchange messages to sync experiment start
        self.human_q.put(self.name + ' ready to start')#signal to the supervisor robot is in position
        go_msg = self.robot_q.get()
        self.Experiment_Logger.write(go_msg)
       
        
#==============================================================================
#         #init variables needed    
        not_moving = {}
        for human in self.settings['humans']:
            not_moving[human] = 0
#         hypothesis_selected = {}
# 
#==============================================================================
        
        #continually grab the output from the plan processes
        #when all 3 plans have sent a message allow them all to do more processing
        #compare results and selct and execute plan
        #check for experiment end conditions to break out of loop, and set end_flag so CE_processes end
        for sim_steps in range(1000):
            start = time.time()

            #select the appropriate hypothesis of human knowledge to be used by all the CEs
            
#==============================================================================
#             for human in self.settings['humans']:#for each human in the experiment
#                 #TODO hypothesis checking, to evaluate estimate of human world knowledge
#                 for hypothesis in enumerate(self.human_knowledge[human]):#for all hypotheses of what that human knows about
#                     #compare human actions with what they would be doing according to each hypothesis and score them according to how likely they are
#                     #plan path for a human from the start position to the assumed target with the assumed obstacle list and compare with actual path taken
#                     #get confidence scores for each hypothesis, deleting hypotheses if they are below a threshold
#                     #select hypothesis for each human
#                     pass
#                 hypothesis_selected[human] = 0#no hypothesis checking yet so just choose the first one
#                 #TODO if it is a different hypothesis from before then update all the human graphs in all CEs
#                 
#==============================================================================
             
            
            if 'DEBUG_position_ROBOT' in self.settings and not self.tracker:
                #currently in debug mode there is no commands to send the robot so just end the process
                #TODO add some output messages here to help with debugging
                #distance_to_target = 0.4
                #if (distance_to_target > 0.1):
                #    self.agent.add_belief('not_at_goal')      
                #self.agent.reason(robot,rule_info)
                self.end_flag.set()#cause CE_processes to terminate on next loop
                for _ in range(3):
                    self.results_q.task_done()

            rule_info = self.ethical_engine.update_beliefs()
                                   
            if not self.end_flag.is_set():
 
                self.ethical_engine.execute_a_rule(robot, rule_info)
                
                end = time.time()
                dur = end - start
                
                try:
                    time.sleep(self.settings['update_rate']-dur)
                except:
                    pass
                for human in self.settings['humans']:#for each human in the experiment
                    if self.tracker.get_speed(human) < self.settings['speed_threshold']:#check human velocity 
                        not_moving[human] = not_moving[human] + 1
                        self.Experiment_Logger.write('human speed ' + str(self.tracker.get_speed(human)))
                    else:
                        not_moving[human] = 0
                    
                
            if self.end_flag.is_set():
                if 'DEBUG_position_ROBOT' not in self.settings:
                    robot.clean_up()
                break#CE thread will run until all robots are stationary or 1000 iteration steps
            
            for _ in range(3):
                self.results_q.task_done()#let the CE processes start on producing the next msg set
        self.end_flag.set()               
        

