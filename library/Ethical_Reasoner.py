#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 18 16:53:09 2017

@author: paul
"""

#this needs a method that is created as a new process in experiment.py 
#when it is run it compares the plan then invokes robot control to execute the next step

class ethical_reasoner():
    def __init__(self,CE_message_q):
        self.message_q = CE_message_q
        #self.robot_controller = ...
        
