#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 18 17:35:24 2017

@author: louiseadennis
"""
import nao_agent
import robot_control
from library import Utilities

class ethical_engine():
    def __init__(self, tracker, robot_q, human_q, session_path, settings, ip_robot = '192.168.20.224'):
        self.Experiment_Logger = Utilities.Logger('ROBOT_control')

        self.agent = nao_agent.Agent()
        self.robot_controller = robot_control.robot_controller()
        # Subject to change
        self.ce = ConsequenceEngine.CE_process()
        
        #believe_at_goal = self.agent.B('at_goal')
        #not_believe_at_goal = self.agent.NOT(self.a)
        
        self.agent.add_condition_rule(self.agent.B('vocal_warning_received'), self.vocal_warning_received_rule)
        self.agent.add_condition_rule(self.agent.B('pointed_warning_received'), self.pointed_warning_received_rule)
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
            self.results_q.task_done()
        robot.speak_text('stop')#debug
        print 'stop'
        
    def stop_moving_rule(self, robot, rule_info):
        self.robot_controller.stop_action(robot, rule_info)
        self.agent.add_belief('stopped_moving')
        
#        Rules for motion
    def movement_rule(self, robot, rule_info):
        self.robot_controller.move_to_target_action(self, robot, rule_info)


