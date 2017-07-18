#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 18 17:01:52 2017

@author: paul
"""

#init the planner - split this between experiemnt.py and the init for this class
#in the init it needs to create a CE instance
#in the main method need to create a sim_eval and run the optimiser

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
            
self.CE_processes = {}
for plan in plan_types:#create processes for each of the 3 plan types to run their own CE and plan optimiser (TODO)
    self.CE_processes[plan]= mp.Process(target=self.CE_process, args=(plan,))
    #self.CE_process[plan].start()
self.results_q = mp.JoinableQueue()#queue for communication between the CE processes and the manager process
self.CE_manager = mp.Process(target=self.CE_manager_proc)#manager process that decides on the plan to execute, and controls the robot

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
                        
                
                