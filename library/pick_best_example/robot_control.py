#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 13:57:06 2017

@author: paul
"""
import time
import ethical_engine

class robot_controller():

    def __init__(self):
        self.name = 'ROBOT'
        self.stop = 0
        self.ethical_engine = ethical_engine.ethical_engine(self)


#    This is the main control loop
    def CE_manager_proc(self):


        while not self.stop:

            rule_info = self.ethical_engine.update_beliefs()

            print self.ethical_engine.agent.beliefbase

            #as rule execution relies on a robot object which doesn't exist in debug mode
            #need an exception to handle debug mode properly
            try:
                self.ethical_engine.execute_a_rule(self, rule_info)
            except:
                self.ethical_engine.execute_a_rule(None, rule_info)


robot_controller = robot_controller();
robot_controller.CE_manager_proc()