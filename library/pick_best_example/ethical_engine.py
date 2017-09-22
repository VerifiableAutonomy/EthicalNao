#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 18 17:35:24 2017

@author: louiseadennis
"""
import nao_agent

class ethical_engine():
    def __init__(self, robot_controller):

        self.agent = nao_agent.NaoAgent()
        self.robot_controller = robot_controller
        self.results_q = []
        self.plan_eval_q = []

        self.stage = 1;

        self.agent.add_condition_rule(self.agent.B('too_close_to_a_human'), self.stop_rule)
        self.agent.add_pick_best_rule(self.agent.AND(self.agent.B('plan'), self.agent.B('fact')), self.compare_plans, self.execute_plan_rule)
        #self.agent.add_pick_best_rule(self.agent.B('plan'), self.compare_plans, self.execute_plan_rule)
        #self.agent.add_condition_rule(self.agent.B('plan'), self.test_rule)
        self.agent.add_rule(self.agent.dummy_rule)
        
    def stop_rule(self,robot,rule_info):
        robot.stop = 1;
        print 'stop_rule'
        
    def test_rule(self,robot,rule_info):
        print 'test_rule'

    def execute_plan_rule(self, plan, robot, robot_rule):
        print 'executing plan ', plan

    def compare_plans(self, plan1, plan2):
        scores = self.agent.belief_value(self.agent.B('scores'))

        if scores[plan1] > scores[plan2]:
            return 1;
        else:
            return 0;

#        update_beliefs populates the agent's belief base and return information necessary for executing rules
    def update_beliefs(self):

        rule_info = {}

        consequence_results = ['p1', 'p2', 'p3']
        scores = {}
        scores['p1'] = 5;
        scores['p2'] = 10;
        scores['p3'] = 7;
        

        self.agent.add_belief_value('plan', consequence_results)
        self.agent.add_belief_value('scores', scores)
        

        if (self.stage == 4):
            self.agent.add_belief('too_close_to_a_human')
        else:
            self.agent.drop_belief('too_close_to_a_human')
            
        if (self.stage == 2):
            self.agent.add_belief('fact')
            
        if (self.stage == 3):  
            self.agent.drop_belief('plan')
        #else:
        #    self.agent.drop_belief('fact')
            
        #print "test ", self.agent.AND(self.agent.B('plan'), self.agent.B('fact'))()
        print "stage ",self.stage
        self.stage = self.stage + 1

        return rule_info
        
    def execute_a_rule(self, robot, rule_info):
        print 'execute a rule'
        self.agent.reason(robot, rule_info)


