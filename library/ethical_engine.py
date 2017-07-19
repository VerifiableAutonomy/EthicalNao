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

class ethical_engine():
    def __init__(self, robot_controller):
        self.Experiment_Logger = Utilities.Logger('ROBOT_control')

        self.agent = nao_agent.Agent()
        self.robot_controller = robot_controller
        self.results_q = self.robot_controller.results_q
        self.settings = self.robot_controller.settings
 
       # Subject to change
        self.ce = ConsequenceEngine.CE_process()
        
        #believe_at_goal = self.agent.B('at_goal')
        #not_believe_at_goal = self.agent.NOT(self.a)
        
        self.agent.add_condition_rule(self.agent.B('vocal_warning_acknowledged'), self.vocal_warning_received_rule)
        self.agent.add_condition_rule(self.agent.B('pointed_warning_acknowledg'), self.pointed_warning_received_rule)
        self.agent.add_condition_rule(self.agent.B('too_close_to_a_human'), self.stop_rule)
        self.agent.add_condition_rule(self.agent.NOT(self.agent.B('human_in_danger')), self.stop_rule)
        self.agent.add_condition_rule(self.agent.B('all_humans_stopped'), self.stop_rule)
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
        self.robot_controller.vocal_warning_action(self, robot, rule_info)
        #if at warn location give warning and signal to human warning given
        current_position = rule_info['current_position']
        self.agent.add_belief('warning_given')
        self.Experiment_Logger.write('Warning given at ' + str(current_position[0]) + ' ' + str(current_position[1]))
        robot.speak_text('vocal warning')#debug
        print 'vocal warning'
        
    def vocal_warning_recieved_rule(self, robot, rule_info):
        self.ce.vocal_warning_received_action(self, robot, rule_info)
        robot.speak_text('vocal warning heard')#debug
        print 'vocal warning heard'
     
    def pointed_warning_rule(self, robot, rule_info):
        self.robot_controller.pointed_warning_action(self, robot, rule_info)
        #if at warn location give warning and signal to human warning given
        current_position = rule_info['current_position']
        self.agent.add_belief('warning_given')
        self.Experiment_Logger.write('Warning given at ' + str(current_position[0]) + ' ' + str(current_position[1]))
        robot.speak_text('pointed warning')#debug
        print 'pointed warning'
        
    def pointed_warning_recieved_rule(self, robot, rule_info):
        self.ce.pointed_warning_received_action(self, robot, rule_info)
        robot.speak_text('pointed warning heard')#debug
        print 'pointed warning heard'
       
        
#       Rules for stopping stuff 
    def stop_rule(self,robot,rule_info):
#         self.end_flag.set()#cause CE_processes to terminate on next loop
        self.robot_controller.stop_action(robot, rule_info)
#         msgs = 0
        for _ in range(3):
            self.robot_controller.results_q.task_done()
        robot.speak_text('stop')#debug
        print 'stop'
        
    def stop_moving_rule(self, robot, rule_info):
        self.robot_controller.stop_action(robot, rule_info)
        self.agent.add_belief('stopped_moving')
        
#        Rules for motion
    def movement_rule(self, robot, rule_info):
        self.robot_controller.move_to_target_action(self, robot, rule_info)
        
#        update_beliefs populates the agent's belief base and return information necessary for executing rules
    def update_beliefs(self):
        self.Experiment_Logger.write('Updating Beliefs')
        
        rule_info = {}
              
        consequence_results = {}
        msgs = 0
            
#        Get messages from Consequence engines
        while msgs < 3:
            result = self.results_q.get()
            consequence_results[result['plan']['type']] = result#store the 3 results in a dictionary keyed by plan type
            msgs = msgs + 1
            
                        
        if consequence_results['move']['inaction_danger']:#if inaction would be dangerous
            self.agent.add_belief('human_in_danger')
            plan_eval = self.settings['MAX_SCORE']#set the current evaluation to max_score
            plan = None
            for consequence_result in consequence_results.values():
                if consequence_result['score'] < plan_eval:
                    plan_eval = consequence_result['score']
                    plan = consequence_result['plan']                        
                                  
        else:#if human not in danger then stop the robot
            self.agent.drop_belief('human_in_danger')
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
            
        if not (self.robot_controller.end_flag.is_set()):
            current_position = self.settings['DEBUG_position_ROBOT']
            self.robot_controller.travelled_path.append(current_position)#log travelled path            

        if not (self.robot_controller.end_flag.is_set()):
            current_position = self.tracker.get_position(self.name)[0:2]
            self.robot_controller.travelled_path.append(current_position)#log travelled path            
 
            rule_info['current_position'] = current_position
            distance_to_target = (numpy.sum((current_position - plan['position'])**2))**0.5
            print 'dist ',distance_to_target
            print current_position
            if (distance_to_target < 0.1):
                self.agent.add_belief('at_goal')
            self.agent.drop_belief('warn_can_be_heard')

        for human in self.settings['humans']:
            current_position_human = self.robot_controller.tracker.get_position(human)[0:2] 
            inter_robot_distance = numpy.sum((current_position - current_position_human)**2)**0.5
            self.Experiment_Logger.write('Distance to ' + human + ' = ' + str(inter_robot_distance))
            if inter_robot_distance < 0.5:#if too close to a human then stop the robot
                self.agent.add_belief('too_close_to_a_human')
                self.Experiment_Logger.write('too close')
                    
            if inter_robot_distance < self.settings['hearing_dist']:
                self.agent.drop_belief('warn_can_be_heard')

        if (not self.human_q.empty()):
            msg = self.human_q.get()
            self.end_flag.set()#debug
            if msg['type'] == 'warn':
                self.agent.add_belief('vocal_warning_acknowledged')
            elif msg['type'] == 'point':
                self.agent.add_belief('visual_warning_acknowledge')
                
        if all(i>7 for i in self.robot_controller.not_moving.values()):
            self.agent.add_belief('all_humans_stopped')


        return rule_info
        
    def execute_a_rule(self, robot, rule_info):
        self.agent.reason(robot, rule_info)


