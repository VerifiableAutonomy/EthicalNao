#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 18 17:35:24 2017

@author: louiseadennis
"""
import nao_agent
import robot_control
import numpy
from library import Utilities
import multiprocessing as mp
import Queue 

class ethical_engine():
    def __init__(self, robot_controller):
        self.Experiment_Logger = Utilities.Logger('Ethical_Eng')

        self.agent = nao_agent.Agent()
        self.robot_controller = robot_controller
        self.results_q = self.robot_controller.results_q
        self.plan_eval_q = self.robot_controller.plan_eval_q
        self.settings = self.robot_controller.settings
        self.msgs = 0
        #default plan of move to objective, plan as class variable so that it remains if there are no new plans
        self.plan = {'type':'move','angle':0,'position':self.settings['ROBOT_objective'],'point_pos':(0,0),'speed':0.25}
        self.consequence_results = None
       # Subject to change
        #self.ce = ConsequenceEngine.CE_process()
        
        #believe_at_goal = self.agent.B('at_goal')
        #not_believe_at_goal = self.agent.NOT(self.a)
        self.agent.add_condition_rule(self.agent.B('too_close_to_a_human'), self.stop_rule)
        self.agent.add_condition_rule(self.agent.B('vocal_warning_acknowledged'), self.vocal_warning_recieved_rule)
        self.agent.add_condition_rule(self.agent.B('pointed_warning_acknowledged'), self.pointed_warning_recieved_rule)
        self.agent.add_condition_rule(self.agent.AND(self.agent.NOT(self.agent.B('human_in_danger')),self.agent.B('new_plan')) , self.own_goal_rule)#TODO the actual outcome should be the robot walks towards its goal
        self.agent.add_condition_rule(self.agent.AND(self.agent.B('human_in_danger'),self.agent.B('new_plan')), self.update_plans_rule)
        self.agent.add_condition_rule(self.agent.AND(self.agent.B('all_humans_stopped'), self.agent.B('at_goal')), self.stop_rule)#TODO this would be own_goal I think, but the human not moving would be identified as not in danger by the planner so the previous rule will cover the situation
        self.agent.add_condition_rule(self.agent.AND(self.agent.B('at_goal'), self.agent.NOT(self.agent.B('stopped_moving'))), self.stop_moving_rule)
        self.agent.add_condition_rule(self.agent.NOT(self.agent.B('at_goal')), self.movement_rule)
        self.agent.add_condition_rule(self.agent.AND(self.agent.AND(self.agent.B('at_goal'), self.agent.B('warn_can_be_heard')),
                                      self.agent.AND(self.agent.NOT(self.agent.B('warning_given')), self.agent.B('warning_plan'))), self.vocal_warning_rule
                                      )
        self.agent.add_condition_rule(self.agent.AND(self.agent.AND(self.agent.B('at_goal'), self.agent.B('warn_can_be_heard')),
                                                     self.agent.AND(self.agent.NOT(self.agent.B('warning_given')), self.agent.B('pointing_plan'))), self.pointed_warning_rule
                                      )
        self.agent.add_rule(self.agent.dummy_rule)
        
#        Rules for Warnings
    def vocal_warning_rule(self, robot, rule_info):
        if not (self.debugging()):
            self.robot_controller.vocal_warning_action(robot, rule_info)
        #if at warn location give warning and signal to human warning given
        current_position = rule_info['current_position']
        self.agent.add_belief('warning_given')
        self.Experiment_Logger.write('Warning given at ' + str(current_position[0]) + ' ' + str(current_position[1]))
        #robot.speak_text('vocal warning')#debug
        print 'vocal_warning_rule'
        
    def vocal_warning_recieved_rule(self, robot, rule_info):
        if not (self.debugging()):
            self.ce.vocal_warning_received_action(robot, rule_info)
        #robot.speak_text('vocal warning heard')#debug
        print 'vocal_warning_recieved_rule'
     
    def pointed_warning_rule(self, robot, rule_info):
        if not (self.debugging()):
            self.robot_controller.pointed_warning_action(robot, rule_info)
        #if at warn location give warning and signal to human warning given
        current_position = rule_info['current_position']
        self.agent.add_belief('warning_given')
        self.Experiment_Logger.write('Warning given at ' + str(current_position[0]) + ' ' + str(current_position[1]))
#        robot.speak_text('pointed warning')#debug
        print 'pointed_warning_rule'
        
    def pointed_warning_recieved_rule(self, robot, rule_info):
        if not (self.debugging()):
            self.ce.pointed_warning_received_action(robot, rule_info)
#        robot.speak_text('pointed warning heard')#debug
        print 'pointed_warning_recieved_rule'
       
        
#       Rules for stopping stuff 
    def stop_rule(self,robot,rule_info):
        self.robot_controller.end_flag.set()#cause CE_processes to terminate on next loop
        if not (self.debugging()):
            self.robot_controller.stop_action(robot, rule_info)
#         msgs = 0
        #for _ in range(3):
        #    self.robot_controller.results_q.task_done()
#        robot.speak_text('stop')#debug
        print 'stop_rule'
        
    def stop_moving_rule(self, robot, rule_info):
        if not (self.debugging()):
            self.robot_controller.stop_action(robot, rule_info)
        self.agent.add_belief('stopped_moving')
        print 'stop_moving_rule'
#        Rules for motion
    def movement_rule(self, robot, rule_info):        
        if not (self.debugging()):
            self.robot_controller.move_to_target_action(robot, rule_info)            
        print 'movement_rule'
        
    def own_goal_rule(self, robot, rule_info):
        self.agent.drop_belief('new_plan')
        self.plan = {'type':'move','angle':0,'position':self.settings['ROBOT_objective'],'point_pos':(0,0),'speed':0.25}
        rule_info['plan'] = self.plan
            
        print 'own_goal_rule'
    def update_plans_rule(self, robot, rule_info):
        self.agent.drop_belief('new_plan')
        consequence_result = self.compare_plans()
        if consequence_result <> None:
            self.plan = consequence_result['plan_params']
        else:#if all the plans fail then robot just walks to its own objective
            self.agent.add_belief('all_plans_fail')#this belief is not used currently but is added for logging purposes
            self.plan = {'type':'move','angle':0,'position':self.settings['ROBOT_objective'],'point_pos':(0,0),'speed':0.25}

        #Helper method for debugging
    def debugging(self):
        return 'DEBUG_position_ROBOT' in self.robot_controller.settings and not self.robot_controller.tracker
        
        
    def compare_plans(self):
        if self.consequence_results == None:
            return None
    
        plan_eval = self.settings['MAX_SCORE']#set the current evaluation to max_score
        result = None
        
        for consequence_result in self.consequence_results:
            print consequence_result['result'].total
#recalculate the total used in the comparison here to allow weights to be set according to the situation
#==============================================================================
#             total = self.settings['W_robot_walking_dist']*consequence_result['result'].robot_walking_dist - \
#                 self.settings['W_danger_distance']*consequence_result['result'].danger_distance + \
#                 self.settings['W_robot_speed']*consequence_result['result'].robot_speed - \
#                 self.settings['W_wait_time']* consequence_result['result'].wait_time - \
#                 self.settings['W_robot_danger_dist']* consequence_result['result'].robot_danger_dist + \
#                 self.settings['W_robot_obj_dist']* consequence_result['result'].robot_obj_dist
#==============================================================================
            if consequence_result['result'].total < plan_eval:
                     plan_eval = consequence_result['result'].total
                     result = consequence_result#['plan_params']
        
        #self.Experiment_Logger.write('Plan selected ' + Utilities.print_dict(plan))

        return result 

#        update_beliefs populates the agent's belief base and return information necessary for executing rules
    def update_beliefs(self):
        self.Experiment_Logger.write('Updating Beliefs')
        
        rule_info = {}
              
        
#==============================================================================
#         msgs = 0#messages from planners
#             
# #        Get messages from planners 
#         self.Experiment_Logger.write('Getting Messages')
#         while msgs < 3:
#             result = self.results_q.get()
#             #consequence_results[result['plan']['type']] = result#store the 3 results in a dictionary keyed by plan type
#             no_intervention = result
#             msgs = msgs + 1
#         self.Experiment_Logger.write('got messages')
#==============================================================================
        
        while not self.results_q.empty():
            result = self.results_q.get()
            no_intervention = result
            self.msgs = self.msgs + 1
        
        if self.msgs == 3:#if there are new plans from the planners then retrieve them   
            self.Experiment_Logger.write('New plan data')
            self.msgs = 0#reset the message counter
            #and Consequence engines
            self.consequence_results = []
            #print 'EE q-size = ',self.plan_eval_q.qsize()
            while True:
                try:
                    self.consequence_results.append(self.plan_eval_q.get(block=False))
                    #print 'loop q-size = ',self.plan_eval_q.qsize()
                except Queue.Empty:
                    #print 'loop exit q-size = ',self.plan_eval_q.qsize()
                    if self.plan_eval_q.qsize() == 0:
                        break
                    
            for _ in range(3):
                    self.results_q.task_done()#let the planner processes start on producing the next msg set
                    
    #==============================================================================
    #         while self.plan_eval_q.qsize():
    #             try:
    #                 consequence_results.append(self.plan_eval_q.get())
    #             except Queue.Empty:
    #                 pass
    #==============================================================================
            
            #print 'EE_2 q-size = ',self.plan_eval_q.qsize()
            #print 'q empty? ',self.plan_eval_q.empty()
            self.Experiment_Logger.write('Plans evaluated = ' + str(len(consequence_results)))
        
#==============================================================================
#         if consequence_results['move']['inaction_danger']:#if inaction would be dangerous
#             self.agent.add_belief('human_in_danger')
#             plan_eval = self.settings['MAX_SCORE']#set the current evaluation to max_score
#             plan = None
#             for consequence_result in consequence_results.values():
#                 if consequence_result['score'] < plan_eval:
#                     plan_eval = consequence_result['score']
#                     plan = consequence_result['plan']                        
#==============================================================================
            #the retrieved plans will include updated human in danger information so update the associated belief
            #and the plan information
            if no_intervention['danger_distance'] > self.settings['safe_dist']:#if human not in danger then stop the robot
                self.agent.drop_belief('human_in_danger')
                #TODO check with Louise if it is okay doing this here rather than inside a rule
                #doing it in a rule would need some careful refactoring to make that work properly
                self.agent.drop_belief('at_goal')
                self.agent.add_belief('new_plan')
                #self.plan = {'type':'move','angle':0,'position':self.settings['ROBOT_objective'],'point_pos':(0,0),'speed':0.25}
                self.Experiment_Logger.write('human not in danger')
            else:           
                self.agent.add_belief('human_in_danger')
                self.agent.add_belief('new_plan')
                #consequence_result = self.compare_plans(consequence_results)
                #if consequence_result <> None:
                #    self.plan = consequence_result['plan_params']
                #else:#if all the plans fail then robot just walks to its own objective
                #    self.agent.add_belief('all_plans_fail')#this belief is not used currently but is added for logging purposes
                #    self.plan = {'type':'move','angle':0,'position':self.settings['ROBOT_objective'],'point_pos':(0,0),'speed':0.25}
                rule_info['closest_danger'] = no_intervention['closest_danger']
                        
            #################### test plan for use in debugging ###############            
            if 'ROBOT_plan' in self.settings: self.plan = self.settings['ROBOT_plan']#predefined set plan from settings file
            ############################################
    
        rule_info['plan'] = self.plan
        self.agent.drop_belief('warning_plan')
        self.agent.drop_belief('pointing_plan')
        if self.plan['type'] == 'warn':
            self.agent.add_belief('warning_plan')                                  
        elif self.plan['type'] == 'point':
            self.agent.add_belief('pointing_plan')
            
        if not (self.debugging()):
            current_position = self.robot_controller.tracker.get_position('ROBOT')[0:2]
        else:
            current_position = self.settings['DEBUG_position_ROBOT']
            
        
        self.robot_controller.travelled_path.append(current_position)#log travelled path            
        distance_to_target = (numpy.sum((numpy.array(current_position) - numpy.array(self.plan['position']))**2))**0.5
        print 'dist ',distance_to_target
        #print current_position
        if (distance_to_target < 0.1):
            self.agent.add_belief('at_goal')
            
        self.agent.drop_belief('warn_can_be_heard')
            
        rule_info['current_position'] = current_position
        rule_info['motion_command'] = self.robot_controller.planners[self.plan['type']].CE.graphs['ROBOT'].motion_command(current_position, numpy.array(self.plan['position']), False)

        for human in self.settings['humans']:
            if not 'DEBUG_position_HUMAN_A' in self.settings:
                current_position_human = self.robot_controller.tracker.get_position(human)[0:2]
            else:
                current_position_human = self.settings['DEBUG_position_HUMAN_A']
 
            inter_robot_distance = numpy.sum((numpy.array(current_position) - numpy.array(current_position_human))**2)**0.5
            self.Experiment_Logger.write('Distance to ' + human + ' = ' + str(inter_robot_distance))
            if inter_robot_distance < 0.5:#if too close to a human then stop the robot
                self.agent.add_belief('too_close_to_a_human')
                self.Experiment_Logger.write('too close')
                
            if inter_robot_distance < self.settings['hearing_dist']:
                self.agent.add_belief('warn_can_be_heard')

        if self.agent.sensor_value('warning_given') == 1:
            if (not self.robot_controller.human_q.empty()):
                msg = self.robot_controller.human_q.get()
                if msg['type'] == 'warn':
                    self.agent.add_belief('vocal_warning_acknowledged')
                elif msg['type'] == 'point':
                    self.agent.add_belief('visual_warning_acknowledge')
                    
        if all(i>3 for i in self.robot_controller.not_moving.values()):
            self.agent.add_belief('all_humans_stopped')
        else:
            self.agent.drop_belief('all_humans_stopped')


        return rule_info
        
    def execute_a_rule(self, robot, rule_info):
        print 'execute a rule'
        self.agent.reason(robot, rule_info)


