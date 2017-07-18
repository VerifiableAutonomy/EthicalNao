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
import numpy
#import warnings
#import threading
#import Queue
import multiprocessing as mp
#from library import HTMLlog
from library import Utilities
#from library import Vicon
from library import Robot
from library import PathPlanning
#from library import easygui
from library import Consequence_para
from library import sim_eval
import GPyOpt
from library import nao_agent
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


        self.agent = nao_agent.Agent()
        
        #believe_at_goal = self.agent.B('at_goal')
        #not_believe_at_goal = self.agent.NOT(self.a)
        
        self.agent.add_condition_rule(self.agent.B('vocal_warning_received'), self.vocal_warning_received_rule)
        self.agent.add_condition_rule(self.agent.B('pointed_warning_received'), self.pointed_warning_received_rule)
        self.agent.add_condition_rule(self.agent.B('too_close_to_a_human'), self.stop_rule)
        self.agent.add_condition_rule(self.agent.NOT(self.agent.B('human_in_danger')), self.stop_rule)
        self.agent.add_condition_rule(self.agent.B('all_humans_stopped'), self.stop_rule)
        self.agent.add_condition_rule(self.agent.AND(self.agent.B('at_goal'), self.agent.NOT(self.agent.B('stopped_moving'))), self.stop_moving_rule)
        self.agent.add_condition_rule(self.agent.NOT(self.agent.B('at_goal')), self.move_to_target_rule)
        self.agent.add_condition_rule(self.agent.AND(self.agent.AND(self.agent.B('at_goal'), self.agent.B('warn_can_be_heard')),
                                      self.agent.AND(self.agent.NOT(self.agent.B('warning_given')), self.agent.B('warning_plan'))), self.vocal_warning_rule
                                      )
        self.agent.add_condition_rule(self.agent.AND(self.agent.AND(self.agent.B('at_goal'), self.agent.B('warn_can_be_heard')),
                                                     self.agent.AND(self.agent.NOT(self.agent.B('warning_given')), self.agent.B('pointing_plan'))), self.pointed_warning_rule
                                      )
        self.agent.add_rule(self.agent.dummy_rule)
        
        
        plan_types = ['move','warn','point']

########create graphs for self and humans in the experiment##############
        self.robot_graph = {}
        self.CE = {}
        for plan in plan_types:
            self.robot_graph[plan] = PathPlanning.Planner(self.tracker)#one graph for each plan type
            self.CE[plan] = Consequence_para.ConsequenceEngine('ROBOT', self.settings['humans'], self.tracker, plan, self.settings, engine_name='CEngine_'+plan)#human_names ['HUMAN_A','HUMAN_B']
            if 'self_obstacles' in self.settings:#if a starting set of objects the robot knows about self_obstacles ['TARGET_A','TARGET_B']
                self.CE[plan].set_obstacles(self.settings['self_obstacles'])

            if 'dangers' in self.settings:#if a starting set of dangers the robot knows about
                #currently dangers just exist in the CE
                self.CE[plan].set_dangers(self.settings['dangers'])#at experiment start the robot assumes humans are ignorant of all dangers so only adds them to its own map
            
            self.CE[plan].set_speed_threshold(self.settings['speed_threshold'])
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
        
        self.CE_processes = {}
        for plan in plan_types:#create processes for each of the 3 plan types to run their own CE and plan optimiser (TODO)
            self.CE_processes[plan]= mp.Process(target=self.CE_process, args=(plan,))
            #self.CE_process[plan].start()
        self.results_q = mp.JoinableQueue()#queue for communication between the CE processes and the manager process
        self.CE_manager = mp.Process(target=self.CE_manager_proc)#manager process that decides on the plan to execute, and controls the robot
        self.end_flag = mp.Event()
    
    
    #RULES
    def move_to_target_rule(self,robot,rule_info):
        #plan and execute robot motion if not at goal
#==============================================================================
#         plan = rule_info['plan']
#         robot.speed_factor = plan['speed']
#         robot_motion = self.CE[plan['type']].motion_command(plan['position'], plan, plot=False)
#         next_position_robot = robot_motion['next_position']
#         self.Experiment_Logger.write('target ' + str(next_position_robot[0])+' '+str(next_position_robot[1]))
#         robot.go_to_position(next_position_robot[0], next_position_robot[1])
#==============================================================================
        robot.speak_text('move to target')#debug
        print 'rule 1'
        
    def stop_moving_rule(self, robot, rule_info):
        #robot.stop_robot()
        self.agent.add_belief('stopped_moving')
    
    def stop_rule(self,robot,rule_info):
#==============================================================================
#         msgs=rule_info['msgs']
#         self.end_flag.set()#cause CE_processes to terminate on next loop
#         robot.stop_robot()
#         msgs = 0
#         for _ in range(3):
#             self.results_q.task_done()
#==============================================================================
        robot.speak_text('stop')#debug
        print 'stop'

    def vocal_warning_rule(self, robot, rule_info):
        #if at warn location give warning and signal to human warning given
        consequence_results = rule_info['consequence_results']
        current_position = rule_info['current_position']
        #robot.stop_robot()
        robot.speak_text(self.settings['warning_call'], blocking=True)
        for human in self.settings['humans']:
            position = consequence_results['warn'][human + '_goal']
            self.robot_q.put({'warning_type' : 'warn', 'position':position})
            self.agent.add_belief('warning_given')
            self.Experiment_Logger.write('Warning given at ' + str(current_position[0]) + ' ' + str(current_position[1]))

        robot.speak_text('vocal warning')#debug
        print 'vocal warning'
        
    def pointed_warning_rule(self, robot, rule_info):
#==============================================================================
#         plan = rule_info['plan']
#         current_position = rule_info['current_position']
#         robot.stop_robot()
#         robot.point_direction_world(plan['point_pos'][0], plan['point_pos'][1])
#         self.Experiment_Logger.write('Pointing given at ' + str(current_position[0]) + ' ' + str(current_position[1]))
#         for human in self.settings['humans']:
#             self.robot_q.put({'warning_type' : 'point','position':plan['point_pos']})
#             self.agent.add_belief('warning_given')
#==============================================================================
        robot.speak_text('point warning')#debug
        print 'point warning'

    def vocal_warning_received_rule(self, robot, rule_info):
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
        robot.speak_text('vocal warning heard')#debug
        print 'vocal warning heard'

    def pointed_warning_received_rule(self, robot):
#==============================================================================
#         update = plan['point_pos']
#                         
#         self.human_knowledge[msg['name']][hypothesis_selected[msg['name']]].append(update)#append the closest danger to the current hypothesis as the warning was ack'd
#         for plan in ['move','warn','point']:
#             for human in self.settings['humans']:
#                 self.CE[plan].add_obstacles(update, actor = human)
#==============================================================================
        robot.speak_text('point warning seen')#debug
        print 'point warning seen'


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
       
        
        #init variables needed    
        not_moving = {}
        #warning_given = {}
        for human in self.settings['humans']:
            not_moving[human] = 0
        hypothesis_selected = {}

        
        #continually grab the output from the plan processes
        #when all 3 plans have sent a message allow them all to do more processing
        #compare results and selct and execute plan
        #check for experiment end conditions to break out of loop, and set end_flag so CE_processes end
        for sim_steps in range(1000):
            start = time.time()

            msgs = 0
            rule_info = {}
            #select the appropriate hypothesis of human knowledge to be used by all the CEs
            
            for human in self.settings['humans']:#for each human in the experiment
                #TODO hypothesis checking, to evaluate estimate of human world knowledge
                for hypothesis in enumerate(self.human_knowledge[human]):#for all hypotheses of what that human knows about
                    #compare human actions with what they would be doing according to each hypothesis and score them according to how likely they are
                    #plan path for a human from the start position to the assumed target with the assumed obstacle list and compare with actual path taken
                    #get confidence scores for each hypothesis, deleting hypotheses if they are below a threshold
                    #select hypothesis for each human
                    pass
                hypothesis_selected[human] = 0#no hypothesis checking yet so just choose the first one
                #TODO if it is a different hypothesis from before then update all the human graphs in all CEs
                
            consequence_results = {}
            while msgs < 3:#keep grabbing msgs until 3 have been grabbed - one from each CE
                result = self.results_q.get()
                consequence_results[result['plan']['type']] = result#store the 3 results in a dictionary keyed by plan type
                msgs = msgs + 1

            rule_info['consequence_results'] = consequence_results
            rule_info['msgs'] = msgs
            #when all 3 msgs received process them
            #the returned results will contain the evaluation score and the plan
            #the scores get compared and the best plan is stored in plan so it can be executed   
            
            if consequence_results['move']['inaction_danger']:#if inaction would be dangerous
                self.agent.add_belief('human_in_danger')
                plan_eval = self.settings['MAX_SCORE']#set the current evaluation to max_score
                plan = None
                for consequence_result in consequence_results.values():
                    if consequence_result['score'] < plan_eval:
                        plan_eval = consequence_result['score']
                        plan = consequence_result['plan']                        
                                  
            else:#if human not in danger then stop the robot
                self.agent.change_belief('human_in_danger', 0)
                self.Experiment_Logger.write('human not in danger')
            
            #################### test plan for use in debugging ###############            
            if 'ROBOT_plan' in self.settings: plan = self.settings['ROBOT_plan']#predefined set plan from settings file
            ############################################
            
            rule_info['plan'] = plan
            self.agent.drop_belief('warning_plan')
            self.agent.drop_belief('pointing_plan')
            if plan['type'] == 'warn':
                      self.agent.add_belief('warning_plan')                                  
            elif plan['type'] == 'point':
                      self.agent.add_belief('pointing_plan')
            
            if 'DEBUG_position_ROBOT' in self.settings and not self.tracker:
                current_position = self.settings['DEBUG_position_ROBOT']
                #currently in debug mode there is no commands to send the robot so just end the process
                #TODO add some output messages here to help with debugging
                #distance_to_target = 0.4
                #if (distance_to_target > 0.1):
                #    self.agent.add_belief('not_at_goal')      
                #self.agent.reason(robot,rule_info)
                self.end_flag.set()#cause CE_processes to terminate on next loop
                msgs = 0
                for _ in range(3):
                    self.results_q.task_done()
            else:
                current_position = self.tracker.get_position(self.name)[0:2]
                rule_info['current_position'] = current_position

            self.travelled_path.append(current_position)#log travelled path            
                        
            
            if not self.end_flag.is_set():
                distance_to_target = (numpy.sum((current_position - plan['position'])**2))**0.5
                print 'dist ',distance_to_target
                print current_position
                if (distance_to_target < 0.1):
                    self.agent.add_belief('at_goal')
                self.agent.drop_belief('warn_can_be_heard')

                for human in self.settings['humans']:
                    current_position_human = self.tracker.get_position(human)[0:2] 
                    inter_robot_distance = numpy.sum((current_position - current_position_human)**2)**0.5
                    self.Experiment_Logger.write('Distance to ' + human + ' = ' + str(inter_robot_distance))
                    if inter_robot_distance < 0.5:#if too close to a human then stop the robot
                        self.agent.add_belief('too_close_to_a_human')
                        self.Experiment_Logger.write('too close')
                    
                    if inter_robot_distance < self.settings['hearing_dist']:
                        self.agent.change_belief('warn_can_be_heard', 1)
                    #distance_to_target = 0#debug
                    
            
                if (self.agent.sensor_value('warning_given') == 1):
                    if (not self.human_q.empty()):
                        msg = self.human_q.get()
                        self.end_flag.set()#debug
                        if msg['type'] == 'warn':
                            self.agent.add_belief('vocal_warning_received')
                        elif msg['type'] == 'point':
                            self.agent.add_belief('visual_warning_received')
                            
                if all(i>7 for i in not_moving.values()):
                    self.agent.add_belief('all_humans_stopped')


                self.agent.reason(robot, rule_info)
                
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
        
    def CE_process(self, plan):
        
        #init variables
        not_moving = {}
        warning_given = {}
        iteration = 0
        for human in self.settings['humans']:
            not_moving[human] = 0
            warning_given[human] = False
        #TODO load a GP, which GP to load is dependent on the plan type and the situation parameters
        while not self.end_flag.is_set():
            #run the CE
            #TODO set the parameters for the plan using the ML framework, to generate values to test, 
            #the following plan evaluation will be called multiple times to use baysian optimisation for the plan class, and the optimised plan passed as a message to the CE_manager process
            
            #pre-defined plan from script file for debugging
            #plan_params = self.settings['ROBOT_plan_' + plan]
            
            
            actor = 'HUMAN_A'
            plot = False#set plotting to false to start with as otherwise there will be too many plots!
            CE = self.CE[plan]
            if 'DEBUG_position_ROBOT' in self.settings:
                robot_location = self.settings['DEBUG_position_ROBOT']
            else:
                robot_location = self.tracker.get_position(self.name)[0:2]

            sim_evaluator = sim_eval.sim_evaluator(actor, plan, plot, CE, robot_location, self.settings)
            
            if 'ROBOT_plan' in self.settings: 
                plan_msg = self.settings['ROBOT_plan']
                plan_msg['type'] = plan
                opt_score = sim_evaluator.calculate_score(numpy.array([[ self.settings['ROBOT_plan']['position'][0],self.settings['ROBOT_plan']['speed'] ]]))
                consequence_results = sim_evaluator.consequence_results
            else:
                #set initial test points apprpriate to the situation
                start = time.time()#debug
                x_quart = abs((sim_evaluator.x_bounds[1]-sim_evaluator.x_bounds[0])/4)
                x_mid = (sim_evaluator.x_bounds[1]+sim_evaluator.x_bounds[0])/2
                x_lower_q = x_mid - x_quart
                x_upper_q = x_mid + x_quart
                #X_initial = numpy.array([(x_lower_q,0.25),(x_mid,0.25),(x_upper_q,0.25)])#only 3 points at the lower quartile point, the mid-point, and upper quartile, all at base speed
                #X_initial = numpy.array([(x_lower_q,0.5),(x_mid,0.5),(x_upper_q,0.5),(x_lower_q,0.1),(x_mid,0.1),(x_upper_q,0.1)])#set 2 
                X_initial = numpy.array([(x_lower_q,0.25),(x_mid,0.25),(x_upper_q,0.25),(x_lower_q,0.5),(x_mid,0.5),(x_upper_q,0.5),(x_lower_q,0.1),(x_mid,0.1),(x_upper_q,0.1)])#set 3 
                
                #will need to test with other initial point sets, incl with different speeds
                bounds =[{'name': 'X', 'type': 'continuous', 'domain': sim_evaluator.x_bounds},
                         {'name': 'speed', 'type': 'continuous', 'domain': (self.settings['min_speed'],self.settings['max_speed'])}]
                plan_opt = GPyOpt.methods.BayesianOptimization(f=sim_evaluator.calculate_score,                 
                                                 domain=bounds,        
                                                 acquisition_type=self.settings['acquisition_type'],
                                                 X = X_initial,
                                                 #exact_feval = True,
                                                 #acquisition_optimizer_type = 'CMA',
                                                 normalize_Y = False,
                                                 acquisition_jitter = 0.01)
                end = time.time()
                init_time = end - start#debug
                #plan_opt.model.model.kern.variance.constrain_fixed(2.5)
                self.Experiment_Logger.write(plan + ' GP init time = ' + str(init_time))
                plan_opt.run_optimization(max_iter=self.settings['max_iter'],verbosity=False)
                opt_time = time.time() - end#debug
                
                self.Experiment_Logger.write(plan + ' GP opt time = ' + str(opt_time) + ' iterations= ' + str(len(plan_opt.X-len(X_initial))))
                
                opt_vals = plan_opt.x_opt
                opt_score = plan_opt.fx_opt[0]
                
                #calculate Y using values stored in sim_evaluator
                #opt_Y = sim_evaluator.calc_Y(opt_vals[0])
                #store all plan values in plan dictionary
                sim_evaluator.calculate_score(numpy.array([opt_vals]))
                #plan_msg = {'type':'move','angle':0,'position':[opt_vals[0],opt_Y],'point_pos':(0,0),'speed':opt_vals[1]}
                plan_msg = sim_evaluator.plan_params
                consequence_results = sim_evaluator.consequence_results
                #if the plan selected is warn or point then updated goal will be set inside the CE, so if it exists replace the plan position with it
                try:
                    plan_msg['position'] = consequence_results['goal']
                except:
                    pass
                #print plan_msg
                #print consequence_results['path']
                #print consequence_results['distances_along_path']
                #store opt_score for transmission
                self.Experiment_Logger.write(plan + ' GP optimal vals = ' + str(plan_msg['position']) + str(plan_msg['speed'])  + ' GP optimal score = ' + str(opt_score))
                self.Experiment_Logger.write(consequence_results['log_msg'])
            #calculate the scores for each hypothesis for each human, and store them in a dictionary
            #consequence_results = {}
            #             self.CE.set_obstacles(self.human_knowledge[human][hypothesis_selected[human]], actor=human)#create graph of current human world view in the CE
            #Calculate CE for each human seperately. TODO extend to include a better way to combine the score. Might need to extend the CE for a plan that includes two humans
#==============================================================================
#             if self.settings['do_plotting']:
#                 plot = self.settings['session_path'] + '/plots/XXX/' + str(iteration) + plan + '_'
#                
#                 
#             for human in self.settings['humans']:
#                 consequence_results[human] = self.CE[plan].predict_all(human, plan_params, plot)#, template=self.settings['plot_template'])#need to modify the CE call to pass it the parameters of the plan
#             
#==============================================================================
            output_msg = {}
            
            #copy the plan parameters into the output msg
            output_msg['plan'] = plan_msg
            #for key,value in plan_params.iteritems():
             #   output_msg[key] = value
            #for human in self.settings['humans']:
            output_msg['HUMAN_A_goal'] = sim_evaluator.current_situation['goal']
                
            #init all result values to zero, they can then be a simple sum of the results from each robot
#==============================================================================
#             output_msg['score'] = 0
#             output_msg['inaction'] = 0
#             for consequence_result in consequence_results.values():
#                 #add the results together somehow - if it is just a single numerical score that's easy. I might want to look at separate score factors that need combining individually and weigthing
#                 try:
#                     output_msg['score'] = output_msg['score'] + consequence_result['score']['total']
#                 except:
#                     print 'o score ',output_msg['score']
#                     print 'cr score ',consequence_result['score']
#                    
#                 output_msg['inaction'] = output_msg['inaction'] + consequence_result['current']['total'] 
#                         
#==============================================================================
                #add the scores to the GP model(s) and evaluate if further optimisation needed
            #currently only testing with 1 human
            output_msg['score'] = opt_score
            output_msg['inaction_danger'] = sim_evaluator.current_situation['in_danger']
            output_msg['inter_rob_dists'] = consequence_results['inter_rob_dists']
            iteration = iteration + 1
            self.results_q.put(output_msg)#put the output msg into the q
            self.results_q.join()#wait until the msgs from all 3 CEs are processed and a new one so notified to proceed
                        
                
                
