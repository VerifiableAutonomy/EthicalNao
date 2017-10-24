#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 18 17:01:52 2017

@author: paul
"""

from library import sim_eval
import multiprocessing as mp
from library import Consequence_para
import GPyOpt
import numpy
import time
import Utilities
#init the planner - split this between experiment.py and the init for this class
#in the init it needs to create a CE instance
#in the plan method need to create a sim_eval and run the optimiser
#in experiment.py 
#init dict of Planners
#planners[plan].plan_process.run()

class Planner():
    def __init__(self, settings, tracker, plan, results_q, plan_eval_q, end_flag):
        self.settings = settings#settings dictionary loaded from file in supervisor and shared with all modules
        self.tracker = tracker#Utilities.Tracker created in supervisor and shared with all modules
        self.plan = plan#plan type for this planner, ['move','warn','point']
        self.results_q = results_q#results_q = mp.JoinableQueue()
        self.end_flag = end_flag#end_flag from robot_controller, used to stop the plan process
        self.CE = Consequence_para.ConsequenceEngine('ROBOT', self.settings['humans'], self.tracker, plan, self.settings, plan_eval_q, engine_name='CEngine_'+plan)
        self.Experiment_Logger = Utilities.Logger(plan + '_planner')
        
        if 'self_obstacles' in self.settings:#if a starting set of objects the robot knows about self_obstacles ['TARGET_A','TARGET_B']
            self.CE.set_obstacles(self.settings['self_obstacles'])

        if 'dangers' in self.settings:#if a starting set of dangers the robot knows about
        #currently dangers just exist in the CE
            self.CE.set_dangers(self.settings['dangers'])#at experiment start the robot assumes humans are ignorant of all dangers so only adds them to its own map
    
        self.CE.set_speed_threshold(self.settings['speed_threshold'])
        self.plan_process = mp.Process(target=self.planning)
        self.Experiment_Logger.write('planner initalised')
        
    def planning(self):
        #main planning process which in a loop calls plan optimiser which in turn invokes the CE
        self.Experiment_Logger.write(str(self.end_flag.is_set()))
        end_flag = False
        while not end_flag:
            plot = False#set plotting to false to start with as otherwise there will be too many plots!
            if 'DEBUG_position_ROBOT' in self.settings:
                robot_location = self.settings['DEBUG_position_ROBOT']                
            else:
                robot_location = self.tracker.get_position('ROBOT')[0:2]

            if 'DEBUG_position_HUMAN_A' in self.settings:
                human_location = self.settings['DEBUG_position_HUMAN_A']
            else:
                human_location = self.tracker.get_position('HUMAN_A')[0:2]

            if 'DEBUG_goal_HUMAN_A' in self.settings:
                human_goal = self.settings['DEBUG_goal_HUMAN_A']
            else:
                human_goal = self.CE.infer_actor_goal('HUMAN_A', minimal_return=True)
                #if human_goal == 'HUMAN_A':
                    #TODO currently this cheats as when it is called the human isn't going fast enough to infer a goal
                    #human_goal=self.settings['HUMAN_A_goal']
            #print 'h goal ',human_goal
            self.Experiment_Logger.write('Human goal ' + str(human_goal))
            #if self.plan == 'move':
            #    print 'hg ',human_goal
            #    print 'hl ',human_location
            if human_goal <> None:
                    sim_evaluator = sim_eval.sim_evaluator('HUMAN_A', self.plan, plot, self.CE, robot_location, human_location, human_goal, self.settings)
                    in_danger = sim_evaluator.current_situation['in_danger']
                    danger_distance = sim_evaluator.current_situation['danger_distance']
                    closest_danger = sim_evaluator.current_situation['closest_danger']
                    path_length = sim_evaluator.current_situation['distances_along_path'][-1]
            else:
                in_danger = False
                danger_distance = 100
                closest_danger = None
            
            self.Experiment_Logger.write('in danger = ' +  str(in_danger) + " danger_dist = " + str(danger_distance))   
            if in_danger:#only run the planner if the human is in danger
                start = time.time()
                if 'ROBOT_plan' in self.settings: 
                    plan_msg = self.settings['ROBOT_plan']
                    plan_msg['type'] = self.plan
                    
                    #start = time.time()
                    opt_score = sim_evaluator.calculate_score(numpy.array([[ self.settings['ROBOT_plan']['position'][0],self.settings['ROBOT_plan']['speed'] ]]))
                    self.Experiment_Logger.write(self.plan + ' plan sim time = ' + str(time.time()-start)) 
                    #consequence_results = sim_evaluator.consequence_results
                    #TODO set plan counter to 1: plan_counter is a shared parameter with the reasoner of how many messages to wait for before starting to compare
                elif 'HEURISTIC_MODE' in self.settings:
                    #calculate the destinations of 3 plausible plans - the midpoint of the human path, a 1/3 and a 2/3 point
                    #print self.plan, ' path ',sim_evaluator.current_situation['path']
                     
                    target_points = [sim_evaluator.current_situation['path'][sim_evaluator.xy_min_idx]]#init target array with this point
                    idx_step_size = int(0.25/sim_evaluator.CE.graphs['HUMAN_A'].get_step())#step in idx needed to get additional points, set to be 0.5m from min point
                    if sim_evaluator.xy_min_idx+idx_step_size <len(sim_evaluator.current_situation['path']):
                        target_points.append(sim_evaluator.current_situation['path'][sim_evaluator.xy_min_idx+idx_step_size])
                        if sim_evaluator.xy_min_idx+2*idx_step_size <len(sim_evaluator.current_situation['path']):
                            target_points.append(sim_evaluator.current_situation['path'][sim_evaluator.xy_min_idx+2*idx_step_size])                    
                    else:
                        target_points.append(sim_evaluator.current_situation['path'][-1])
                        
                    for target in self.settings['tracked_targets']:#add all the target points as potential goals
                        target_points.append(self.tracker.get_position(target)[0:2])
                        
                    for target_point in target_points:
                        print target_point
                        opt_score = sim_evaluator.calculate_score(numpy.array([[ target_point[0], target_point[1] ]]))
                        
#==============================================================================
#                     for i in range(2,5):
#                         #print self.plan, "len ", i, " = ", int(len(sim_evaluator.current_situation['path'])*(i/6.0))
#                         
#                         path_idx = int(len(sim_evaluator.current_situation['path'])*(i/6.0))
#                         #print self.plan, i, " ", sim_evaluator.current_situation['path'][path_idx][0], sim_evaluator.current_situation['path'][path_idx][1]
#                         opt_score = sim_evaluator.calculate_score(numpy.array([[ sim_evaluator.current_situation['path'][path_idx][0], sim_evaluator.current_situation['path'][path_idx][1] ]]))
#                         self.Experiment_Logger.write(str(sim_evaluator.current_situation['path']))
#==============================================================================
                    self.Experiment_Logger.write(self.plan + 'HM plan sim time = ' + str(time.time()-start)) 
                else:
                    #set initial test points apprpriate to the situation
                    #start = time.time()#debug
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
                    self.Experiment_Logger.write(self.plan + ' GP init time = ' + str(init_time))#TODO sort logging out
                    plan_opt.run_optimization(max_iter=self.settings['max_iter'],verbosity=False)
                    opt_time = time.time() - end#debug
                    
                    self.Experiment_Logger.write(self.plan + ' GP opt time = ' + str(opt_time) + ' iterations= ' + str(len(plan_opt.X)))
                    #print 'iter ',len(plan_opt.X)
                    #TODO decide if we want the planner to return what it thinks is the optimal plan
                    #opt_vals = plan_opt.x_opt
                    #opt_score = plan_opt.fx_opt[0]                
                        
            end_flag = self.end_flag.is_set()
                #self.results_q.put(sim_evaluator.current_situation)#number of tested plans is returned via results_q to ethical_engine to say that planning has ended and how many plans there are to compare
            #always send a results message even if no planning happened so the ethical engine knows whether or not the human is in danger
            #print self.plan
            self.results_q.put({'danger_distance':danger_distance,'closest_danger':closest_danger, 'path_length':path_length})
            #self.results_q.put(output_msg)#put the output msg into the q
            self.results_q.join()#wait until the msgs from all 3 CEs are processed and a new one so notified to proceed
            self.Experiment_Logger.write('q joined')                

#==============================================================================
# self.robot_graph = {}
# self.CE = {}
# for plan in plan_types:
#     self.CE[plan] = Consequence_para.ConsequenceEngine('ROBOT', self.settings['humans'], self.tracker, plan, self.settings, engine_name='CEngine_'+plan)#human_names ['HUMAN_A','HUMAN_B']
#     if 'self_obstacles' in self.settings:#if a starting set of objects the robot knows about self_obstacles ['TARGET_A','TARGET_B']
#         self.CE[plan].set_obstacles(self.settings['self_obstacles'])
# 
#     if 'dangers' in self.settings:#if a starting set of dangers the robot knows about
#         #currently dangers just exist in the CE
#         self.CE[plan].set_dangers(self.settings['dangers'])#at experiment start the robot assumes humans are ignorant of all dangers so only adds them to its own map
#     
#     self.CE[plan].set_speed_threshold(self.settings['speed_threshold'])
#             
# self.CE_processes = {}
# for plan in plan_types:#create processes for each of the 3 plan types to run their own CE and plan optimiser (TODO)
#     self.CE_processes[plan]= mp.Process(target=self.CE_process, args=(plan,))
#     #self.CE_process[plan].start()
# self.results_q = mp.JoinableQueue()#queue for communication between the CE processes and the manager process
# self.CE_manager = mp.Process(target=self.CE_manager_proc)#manager process that decides on the plan to execute, and controls the robot
# 
#     def CE_process(self, plan):
#==============================================================================
        
        #init variables
#==============================================================================
#         not_moving = {}
#         warning_given = {}
#         iteration = 0
#         for human in self.settings['humans']:
#             not_moving[human] = 0
#             warning_given[human] = False
#         #TODO load a GP, which GP to load is dependent on the plan type and the situation parameters
#         while not self.end_flag.is_set():
#             #run the CE
#==============================================================================
            #TODO set the parameters for the plan using the ML framework, to generate values to test, 
            #the following plan evaluation will be called multiple times to use baysian optimisation for the plan class, and the optimised plan passed as a message to the CE_manager process
            
            #pre-defined plan from script file for debugging
            #plan_params = self.settings['ROBOT_plan_' + plan]
            
            
            
                
                