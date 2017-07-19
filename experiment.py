#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 27 11:17:31 2017

@author: paul
"""

import shutil
import os
import sys
import scipy.io as io
import matplotlib.pyplot as pyplot
import numpy
#import warnings
#import threading
#import Queue
import getopt
import time
import multiprocessing as mp
import ast

from library import HTMLlog
from library import Utilities
from library import Vicon
from library import Robot
from library import PathPlanning
from library import Webcam
from library import Utilities
from library import human_control
from library import robot_control

#experiment script
#call with arg of experiment name (-s <file name>), this will determine the parameters file to load
#load parameter file
#check the instance of log files already existing and add an appropriate ID to the end of the log file names
#create logger
#create map of arena at experiment start
#create objects: tracker, human(s), robot, camera handler



def main(argv):
    settings_file = ''
    try:
        opts, args = getopt.getopt(argv,"s:e:p")
        #print(opts)
    except getopt.GetoptError:
        print 'experiment.py -s <script_settings_file> -e <optional_experiment_number> -p <if plotting should happen>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == "-s":
            settings_file = arg
            print 'Script file is ', settings_file
        elif opt == "-e":
            expID = arg
            print 'Exp ID is ', expID
        elif opt == "-p":
            plotting = True
   
   
   
    settings = {}
    try:
        with open(settings_file) as f:
                for line in f:
                    if '#' not in line:#hash used in settings file to comment lines out
                        try:
                            (key, val) = line.split()                        
                        except:
                            print line
                            sys.exit(2)
                        try:
                            val = ast.literal_eval(val)#val is processed as if it is python code not a string, so ensure settings file correctly formatted
                        except:
                            print val#if not valid formatting print the eroneous value for debugging purposes
                        if isinstance(val, basestring):
                            if ';' in val:
                                val = val.replace(';',' ')#put spaces in the warning message string
                        settings[key] = val
    except IOError:
        print 'invalid file'
        sys.exit(2)
           

##################################    
#prepare folders and files    
##################################
    
    pyplot.close('all')
    pyplot.style.use('ggplot')

    #dir structure created for every experiment
    session_path = 'data/' + settings['session_name']
    data_present = os.path.isdir(session_path)
   
    if not data_present:
        #make a new experiment folder
        Utilities.empty_folder(session_path)
        #make a new folder tree for exp 1
        session_path = session_path + '/' + settings['session_name'] + '_1'
        Utilities.make_experiment_tree(session_path)       
    else:
        #get the directories 
        dirs = os.walk(session_path).next()[1]#creates a list of the dirs in the session path, i.e., experiment trials
        #check for highest exp number and add one to create base folder name
        trial = 0
        for d in dirs:
            if int(d.split('_')[-1]) > trial:
                trial = int(d.split('_')[-1])#last part of folder name is the experiment trial number
        #make a tree at the base folder name
        session_path = session_path + '/' + settings['session_name'] + '_' + str(trial+1)
        Utilities.make_experiment_tree(session_path)
    
    #remove all old log files as they get created in the main script folder but then copied to the specific session folder at the end of the exp
    files = os.listdir('.')
    for f in files:
        if 'Log' in f:
            Utilities.remove_stable(f)
        
    
    #ce_log = 'data/' + session_name + '/logs/ce_log.html'
    settings['session_path'] = session_path

#####################
# create objects
#####################

    Experiment_Logger = Utilities.Logger('Supervision')
    
    Experiment_Logger.write('Running script: ' + settings['session_name'])

    Experiment_Logger.write('Generating camera object')
    
    #Camera = Webcam.Webcam()

    #create managed dicts the handles for which can be passed to all processes that need access to the tracker
    #create and start a process that repeatedly polls the tracker for information on all tracked objects and stores that data in the shared dicts
    
    Experiment_Logger.write('Generating tracker process')
    #create empty sharable dictionaries
    shared_d_p = mp.Manager().dict({})
    shared_d_v = mp.Manager().dict({})
    shared_d_r = mp.Manager().dict({})
    shared_d_s = mp.Manager().dict({})
    tracked_objects = settings['tracked_targets'] + settings['humans'] + ['ROBOT']
    #create the sharable tracker
    if 'DEBUG_goal_HUMAN_A' in settings:
        Tracker = None
    else:
        Tracker = Utilities.Tracker(tracked_objects, settings['virtual_objects'], shared_d_p, shared_d_v, shared_d_r, shared_d_s)    
        while 'ROBOT' not in shared_d_p.keys(): pass#wait for tracker dict to fill
            
        
#==============================================================================
#     danger_locs =[]
#     for danger in settings['dangers']:
#         if isinstance(danger,basestring):
#             danger_locs.append(Tracker.get_position(danger)[:2])
#         else:
#             danger_locs.append(danger[:])
#     
#     danger_dists = Utilities.distance2points(Tracker.get_position('HUMAN_A')[:2], danger_locs)
#     danger_dists = numpy.array(danger_dists)
#     min_index = numpy.argmin(danger_dists)
#     min_danger = danger_locs[min_index]    
#     print min_danger
# 
#     #print Tracker.enquire('ROBOT')
#     Tracker.stop()
#     sys.exit(2)
#     
#==============================================================================

    #to avoid issues of communication failure warning and pointing, and acknowledgement is signalled internally using message queues
    #this also gives complete control over whether warning/point will be noticed
    robot_q = mp.Queue()
    human_q = mp.Queue()
    
    #create the robot object
    Experiment_Logger.write('Generating Ethical Robot')
    robot=robot_control.robot_controller(Tracker,robot_q, human_q, session_path, settings)
    #start the CE manager process,- this will cause the robot to walk to it's starting place and block until it gets there
    #it then sits and waits for messages from the CE processes so can be safely started before the CE processes
    #it compares the optimal plan suggested by all the CE processes and selects which to execute and instruct ther robot
    robot.CE_manager.start() 
        
    #do the same for the human(s)
    if 'DEBUG_goal_HUMAN_A' not in settings:
        Experiment_Logger.write('Generating Human Robots')
        humans = {}
        for human in settings['humans']: humans[human] = human_control.human_controller(Tracker, robot_q, human_q, session_path, settings,  name=human)
        for human in settings['humans']: humans[human].plan_process.start()
        
        for _ in range(len(settings['humans'])+1):#for all humans + ethical robot 
            msg = human_q.get()#wait for all actors to signal exeriment ready
            Experiment_Logger.write(msg)

    ################ debug ####################
    #TODO reconfigure this bit of debug code to run the test exps without a tracker
    #as it only needs one iteration, can fake the human messages and not run the human controller
    #need to ensure that there is log information on the progress of the optimiser including both results and times
    #should maybe run optimisers with a range of iteration limits to test optimality and timing with various settings
    
#==============================================================================
#     robot_q.put({'warning_type' : 'debug'})
#    for human in settings['humans']: humans[human].plan_process.start()
#     debug_msg = human_q.get()
#     print debug_msg
#     for human in settings['humans']: humans[human].plan_process.join()
#     sys.exit(2)
#==============================================================================

#    robot.CE_manager.start()#start the manager process, it sits and waits for messages so gets started before the CE processes
    
#    for plan in ['move','warn','point']: robot.CE_processes[plan].start()#start the CE processes
    #robot.CE_processes['move'].start()
#==============================================================================
#     for human in settings['humans']:
#         warning_msg = robot_q.get()
#         ack_msg = {'name':human, 'msg': warning_msg['warning_type'] + '_ack'}
#         human_q.put(ack_msg)
#         print warning_msg
#==============================================================================
    if 'DEBUG_goal_HUMAN_A' in settings:
        debug_msg = human_q.get()#get the robots ready to start message
        print debug_msg
        robot_q.put('DEBUG_GO')#send start message to the robot controller
        human_q.put({'type':'warn'})        
        CE_processes = {}
        for plan in ['move','warn','point']: CE_processes[plan] = mp.Process(target=robot.CE_process, args=(plan,))
        for plan in ['move','warn','point']: CE_processes[plan].start()

        #for _ in range(3):
        #    robot.results_q.put(1)
        robot.CE_manager.join()
        
        
        Experiment_Logger.write('Loop exited')
             
        for f in files:
            if 'Log' in f:
                shutil.copy(f, session_path + '/logs/')
        sys.exit(2)#only run to here in debug mode
    ###########################################

######################################
# Plot Arena Layout
######################################
    
    robot_location = Tracker.get_position('ROBOT')
   
    human_locations = {}
    for human in settings['humans']:
        human_locations[human] = Tracker.get_position(human)
    targets = {}
    for tracked_target in settings['tracked_targets']:
        targets[tracked_target] = Tracker.get_position(tracked_target)
    
    pyplot.figure()
    pyplot.hold(True)
    pyplot.scatter(robot_location[0], robot_location[1], s=150, c='b')
    pyplot.annotate('ROBOT', robot_location[:2])
    
    for human in settings['humans']:    
        pyplot.scatter(human_locations[human][0], human_locations[human][1], s=150, c='r')
        pyplot.annotate(human, human_locations[human][:2])

    
    for human in settings['humans']:
        pyplot.scatter(settings[human+'_start'][0], settings[human+'_start'][1], s=150, c='r', alpha=0.5)
    
    pyplot.scatter(settings['ROBOT_start'][0], settings['ROBOT_start'][1], s=150, c='b', alpha=0.5)
    
    for target_name, tracked_target in targets.iteritems():
        pyplot.scatter(tracked_target[0], tracked_target[1], s=150, c='grey')    
        pyplot.annotate(target_name, tracked_target[:2])

    
    pyplot.xlim([-2, 2])
    pyplot.ylim([-2, 2])
    pyplot.hold(False)
    pyplot.savefig(session_path + '/plots/Arena_Layout.png')
    pyplot.close('all')
    
    #Camera.start_capture(session_path + '/images/', rate=settings['camera_rate'], prefix='overhead_view_')

#########################  
# Run the experiment
#########################

    #all robots should now be in position so start them going
    for plan in ['move','warn','point']: robot.CE_processes[plan].start()#start the CE processes
    for _ in range(len(settings['humans'])+1): robot_q.put('GO!')    

    #Wait for all robot control processes to end before grabbing log data
    for human in settings['humans']: humans[human].plan_process.join()
    for plan in ['move','warn','point']: robot.CE_processes[plan].join()
    robot.CE_manager.join()
    
    
    Experiment_Logger.write('Loop exited')
    #Camera.stop_capture()
    Tracker.stop()
    #HTMLlog.log2html('Default_Log.txt')#converting the log text files to html now happens in a separate script
    experimental_data = {}
    experimental_data['time'] = time.asctime()
    
    for target_name in settings['tracked_targets']:
        experimental_data[target_name] = Tracker.get_position(target_name)
        
    experimental_data['robot_path'] = robot.travelled_path
    for human in settings['humans']:
        experimental_data[human + '_path'] = humans[human].travelled_path
    
    io.savemat(session_path + '/experimental_data.mat', experimental_data)
    shutil.copy(settings_file, session_path + '/code/')
    for f in files:
        if 'Log' in f:
            shutil.copy(f, session_path + '/logs/')
    
if __name__ == "__main__":
   main(sys.argv[1:])