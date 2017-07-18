#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu May 18 17:14:42 2017

@author: paul
"""
import Utilities
import numpy as np

#wrapper for predict all to accept and return numpy arrays

#input numpy array needs to be turned into a dict of values, 
#for intercept need to calculate the Y value from the suggested X to make sure an intercept happens
#also must calculate acceptable range in which X can be suggested to ensure intercept
#call predict all
#store results in a class variable so they can be grabbed from outside the wrapper class
#convert results from predict all into a probability of success, made up of a (weighted?) average of failure probabilities
#calc probabilities from the returned score components
#all the bits that currently work on the output score will need updating to accept the new way of doing things

#run predict path for human
#use Utilities.distance2points(robot_location, human_trajectory) to get distance between robot and human predicted path
#also need human_trajectory['distances_along_path'] to get human travel
#calculate min and max point on path where robot has to walk human distance * max speed ratio to get X domain
ROBOT_MAX_S = 0.5

class sim_evaluator():
    def __init__(self, actor, plan, plot, CE, robot_location, settings):
        self.actor = actor#which human is being evaluated
        self.plot = plot#plot file string
        self.CE = CE#pass in the CE that will be called by the wrapper class
        self.plan = plan#plan type to be evaluated, a CE object only evaluates plans of one type
        self.settings = settings
        self.consequence_results = None
        self.plan_params = {}
        #self.max_score = max_score#pass in the max_score that corresponds to a failed plan
        if 'DEBUG_goal_HUMAN_A' in self.settings:
            self.current_situation = CE.predict_and_evaluate(actor, goal=self.settings['DEBUG_goal_HUMAN_A'], plot=plot)#.replace('XXX', actor)+actor)
        else:
            #TODO currently this cheats as when it is called the human isn't going fast enough to infer a goal
            self.current_situation = CE.predict_and_evaluate(actor, goal=self.settings[self.actor + '_goal'], plot=plot)#.replace('XXX', actor)+actor)
        #calculate the max and min points on the human path that the robot can reach at max speed to set the bounds for the GP
        #for all plans this is slightly more restrictive than needed as robot won't need to walk all the way to the human path to succeed, esp. with warn and point
        distances_to_path = Utilities.distance2points(np.array(robot_location), self.current_situation['path'])
        speed_ratio = ROBOT_MAX_S/0.25
        x_lim = []
        for idx,distance in enumerate(distances_to_path):
            if distance <= self.current_situation['distances_along_path'][idx]*speed_ratio:
                x_lim.append(self.current_situation['path'][idx][0])#append valid X values to the current list
        
        #search x_lims to find range of values for x
        x_lim = np.array(x_lim)
        self.x_bounds = (np.min(x_lim),np.max(x_lim))
        #use path to estimate line parameters so Ys can be calculated from suggested Xs
        #print self.current_situation['path']
        if self.current_situation['path'][0][1] == self.current_situation['path'][-1][1]:
            #if the y values are equal
            self.grad = 0 #grad is zero and y = constant (intercept)
            self.intercept = self.current_situation['path'][0][1]
        elif self.current_situation['path'][0][0] == self.current_situation['path'][-1][0]:
            #if the x values are equal
            self.grad = 'X' #grad is zero and x = constant (intercept), flag value to indicate optimisation changed
            self.intercept = self.current_situation['path'][0][1]      
        else:      
            self.grad = (self.current_situation['path'][0][1] - self.current_situation['path'][-1][1])/(self.current_situation['path'][0][0] - self.current_situation['path'][-1][0])
            self.intercept = self.current_situation['path'][0][1] - (self.grad*self.current_situation['path'][0][1])
            
    def calculate_score(self, params):
        #params is a numpy array of the X [0] and speed [1] values to be used in the evaluation
        self.plan_params = {}
        #print params
        self.plan_params['type'] = self.plan
        if 'ROBOT_plan' in self.settings:
            self.plan_params['position'] =self.settings['ROBOT_plan']['position']
        elif self.grad == 'X':#grad set to X to indicate X is constant in path sp parameter is just the Y value
            self.plan_params['position'] = [self.intercept,params[0][0]]
        else:
            try:
                self.plan_params['position'] = [params[0][0],self.calc_Y(params[0][0])]
            except:
                print params
                        
        self.plan_params['point_pos'] = (0,0)#currently fixed for testing but can be added as optimisation parameters
        self.plan_params['speed'] = params[0][1]

        human = 'HUMAN_A'#assumes both humans are going at the same speed. TODO remove the need for this assumtion
        human_speed = self.settings[human + '_speed_factor']
        
        if(self.plan_params['speed']<human_speed):#if the robot will be slower
            self.CE.change_step(human,0.25)#set the human to the default step size as it is faster
            step = 0.25 * (self.plan_params['speed']/human_speed)#calculate smaller step size
            self.CE.change_step('ROBOT',step)#reduce the step size for the robot
        elif(self.plan_params['speed']>human_speed):#if the robot will be faster
            self.CE.change_step('ROBOT',0.25)#set the robot to the default step size as it is faster
            step = 0.25 * (human_speed/self.plan_params['speed'])#calculate smaller step size
            self.CE.change_step(human,step)#reduce the step size for the robot   

        robot_plan = self.CE.predict_path('ROBOT', goal=self.plan_params['position'], plot=False)
        
        if self.plan <> 'move':
            dists_to_path = Utilities.distance2points(self.plan_params['position'], robot_plan['path'])
            try:
                stop_idx = next(idx for idx,dist_to_path in enumerate(dists_to_path) if dist_to_path < self.settings['hearing_dist'])
                self.plan_params['position'] = robot_plan['path'][stop_idx]
                robot_plan['path'] = robot_plan['path'][:stop_idx+1]#crop path so it ends on stop_idx
                robot_plan['distances_along_path'] = robot_plan['distances_along_path'][:stop_idx+1]
            except:#if no intercept it will raise an exception and set the search return to None
                pass#this should never happen as robot path will enable it to get within earshot guaranteed, but if it can't position will be unchanged
        #have the CE calculate the score
        self.consequence_results = self.CE.predict_all(self.actor, self.plan_params,Robot_Plan=robot_plan, plot=self.plot)
        
#==============================================================================
#         if consequence_results['total'] == self.max_score:
#             
#         #convert the returned score into a probability of failure
#         failure_probs ={}
#         failure_probs['distance'] = None#further walked = more likely to fail
#         failure_probs['speed'] = None#faster walked = more likely to fail
#         failure_probs['danger_h'] = None#closer to danger the more likely for the human to miss the warning and hit the danger
#         failure_probs['danger_r'] = None#closer to danger the more likely an error causes a problem
#         failure_probs['wait'] = None#the longer the wait the less likely to fail
#==============================================================================
        
        return np.array([self.consequence_results['score']['total']])

    def calc_Y(self, X):
        return self.grad*X+self.intercept
