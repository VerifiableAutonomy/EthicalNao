import matplotlib.pyplot as pyplot
import Utilities
import PathPlanning
import copy
import Rotate
import numpy
import time
#import sys

Utilities.set_numpy_style()
MAX_DIST = 2


def get_value(distance):
    return Utilities.sigmoid(distance, threshold=1, slope=5)


def plot_value_function():
    handle = pyplot.figure()
    xi = numpy.linspace(0, 1.5)
    yi = get_value(xi)
    pyplot.plot(xi, yi)
    pyplot.xlabel('Distance')
    pyplot.ylabel('Value')
    return handle


class ConsequenceEngine():
    def __init__(self, self_name, actor_names, tracker, plan, settings, engine_name='CEngine'):
                        
        self.__logger = Utilities.Logger(engine_name)
        self.__logger.write('Creating consequence engine ' + engine_name + ' for ' + self_name)

        self.__safe_distance = 0.30
        self.__speed_threshold = 0.01

        self.__tracker = tracker
        self.__self_name = self_name
        self.__engine_name = engine_name
        self.__actor_names = actor_names
        self.__graphs = {}

        self.__dangers = []
        self.__danger_locations = None
        
        self.__plan = plan
        self.settings = settings

        for actor in actor_names: self.make_graph(actor)
        self.make_graph(self_name)

        
    def set_speed_threshold(self, threshold):
        self.__speed_threshold = threshold

    def make_graph(self, actor, data=False, step=None):
        graph = PathPlanning.Planner(self.__tracker, data=data, step=step)
        self.__graphs[actor] = copy.copy(graph)
    
    def motion_command(self, goal, plan, plot=False):
        g = self.__graphs[self.__self_name]
        start = self.__tracker.get_position(self.__self_name)[0:2]
        result = g.motion_command(start, goal, plot)
        return result

    def set_dangers(self, dangers):
        if type(dangers) is not list: dangers = [dangers]
        locations = []
        for danger in dangers:
            if type(danger) == str: danger = self.__tracker.get_position(danger)
            danger = danger[:2]
            locations.append(danger)
        locations = numpy.array(locations)
        locations = numpy.reshape(locations, (-1, 2))
        self.__danger_locations = locations
        self.__dangers = dangers[:]
        self.add_obstacles(dangers, 'self')

    def add_obstacles(self, obstacles, actor=None):
        if type(obstacles) is not list: obstacles = [obstacles]
        if actor == 'self': actor = self.__self_name
        g = self.__graphs[actor]
        current_obstacles = g.get_obstacles()
        new_obstacles = current_obstacles + obstacles
        self.set_obstacles(new_obstacles, actor)

    def set_obstacles(self, obstacles, actor=None):
        if actor is None: actor = self.__self_name
        if actor == 'self': actor = self.__self_name
        if not type(obstacles) == list: obstacles = [obstacles]
        obstacles = obstacles[:]
        g = copy.copy(self.__graphs[actor])
        g.set_obstacles(obstacles)
        self.__graphs[actor] = copy.copy(g)
        
    def change_step(self, actor, step):
        self.__graphs[actor].set_step(step)

    def world2actor(self, agent_name, world_x, world_y):
        '''
        :param agent_name:
        :param world_x:
        :param world_y:
        :return:
        '''
        rotation_agent = self.__tracker.get_rotation(agent_name)
        robot_position = self.__tracker.get_position(agent_name)
        robot_position[2] = 0
        angle = rotation_agent[2]  # this is the angle detected by the vicon - not taking into account the head yaw
        world_cds = numpy.array([world_x, world_y, 0])
        relative = world_cds - robot_position
        relative = Rotate.rotate_z(-angle, relative)
        relative = relative[0]
        relative = numpy.asarray(relative)
        relative = relative.flatten()
        angle = numpy.arctan2(relative[1], relative[0])
        relative_angle = numpy.rad2deg(angle)
        result = numpy.array([relative[0], relative[1], relative_angle])
        return result

    def infer_actor_goal(self, actor, minimal_return=False):
        objects = copy.copy(self.__tracker.all_objects)
        objects.remove(self.__self_name)
        for a in self.__actor_names: objects.remove(a)  # actors cannot be goals
        position_actor = self.__tracker.get_position(actor)
        velocity_actor = self.__tracker.get_velocity(actor)
        speed_actor = numpy.linalg.norm(velocity_actor)
        angles = []
        for object_name in objects:
            object_position = self.__tracker.get_position(object_name)
            world_x = object_position[0]
            world_y = object_position[1]
            result = self.world2actor(actor, world_x, world_y)
            angle = abs(result[2])
            angles.append(angle)

        angles = numpy.array(angles)
        index = numpy.argmin(angles)
        inferred_goal = objects[index]

        if speed_actor < self.__speed_threshold: inferred_goal = actor
        if minimal_return: return inferred_goal

        velocity_actor_norm = Utilities.normalize(velocity_actor) * 0.25
        velocity_actor_norm = velocity_actor_norm + position_actor
        velocity_actor_plot = numpy.vstack((position_actor, velocity_actor_norm))
        self.__logger.write('Inferred Goal for ' + actor + ': ' + str(inferred_goal))

        result = {}
        result['inferred_goal'] = inferred_goal
        result['velocity_agent_norm'] = velocity_actor_norm
        result['velocity_agent_plot'] = velocity_actor_plot
        result['position_agent'] = position_actor
        result['velocity_agent'] = velocity_actor
        result['speed_agent'] = speed_actor
        result['angles'] = angles
        result['objects'] = objects
        return result

    def predict_path(self, actor, goal=None, plot=False):
        if goal is None: goal = self.infer_actor_goal(actor, minimal_return=True)
        
        if type(goal) == str: self.__tracker.get_position(goal)
        
        if 'DEBUG_position_' + actor in self.settings:
            start = self.settings['DEBUG_position_' + actor]
        else:
            start = self.__tracker.get_position(actor)
            start = numpy.array(start[:2])
        
        #start = [0,0]#debug start actor at 0,0
        g = self.__graphs[actor]
        result = g.find_path(start, goal, plot)
        result['actor'] = actor
        result['start'] = start
        result['goal'] = goal
        path = numpy.vstack((result['start'], result['path']))#result['path']
        distances_along_path = numpy.diff(path, n=1, axis=0)
        #distances_along_path = numpy.vstack((numpy.array([0, 0]), distances_along_path))
        distances_along_path = numpy.sum(distances_along_path ** 2, axis=1) ** 0.5
        distances_along_path = numpy.cumsum(distances_along_path)
        result['distances_along_path'] = distances_along_path
        if plot:
            if type(goal) == str: goal = self.__tracker.get_position(goal)
            dangers = self.__danger_locations
            pyplot.hold(True)
            pyplot.scatter(dangers[:, 0], dangers[:, 1], s=500, c='red', alpha=0.5, zorder=10)
            pyplot.scatter(goal[0], goal[1], s=250, c='green', alpha=0.5, zorder=10)
            pyplot.hold(False)
        if type(plot) == str: pyplot.savefig(plot + '_env.png')
        return result

    def evaluate_path(self, result):
        #this function is for evaluating the path of the humans only!
        #it only uses proximity to dangers as other criteria relate only to robot actions: costs, chance of success etc.
        trajectory = result['path']
        locations = self.__danger_locations
        nr_dangers = len(self.__dangers)
        distances = []
        for index in range(0, nr_dangers):
            danger_location = locations[index, :]
            current_distances = Utilities.distance2points(danger_location, trajectory)
            distance = numpy.min(current_distances)
            distances.append(distance)

        distances = numpy.array(distances)
        min_index = numpy.argmin(distances)
        min_danger = self.__danger_locations[min_index]#self.__dangers[min_index]
        min_dist = distances[min_index]

        evaluation = {}
        evaluation['actor'] = result['actor']
        evaluation['in_danger'] = False
        evaluation['dangers'] = self.__dangers
        evaluation['min_allowable_distance'] = self.__safe_distance
        evaluation['closest_danger'] = min_danger
        evaluation['danger_distance'] = min_dist
        evaluation['value'] = get_value(min_dist)
        if min_dist < self.__safe_distance: evaluation['in_danger'] = True
        #evaluation['total'] = MAX_DIST - self.settings['W_danger_distance']*evaluation['danger_distance']
        return evaluation

    def predict_and_evaluate(self, actor,goal=None, plot=False, write_output=False):
        #only for use by the human - the robot has seperate functions depending on plan
        start = time.time()
        prediction = self.predict_path(actor, goal, plot)
        evaluation = self.evaluate_path(prediction)
        text = 'RESULT P&E: ' + prediction['actor'] + ', '
        text += str(prediction['goal']) + ', '
        text += str(evaluation['closest_danger']) + ', '
        text += Utilities.num2str(evaluation['danger_distance']) + ', '
        text += str(evaluation['in_danger']) + ', '
        if write_output: self.__logger.write(text)
        end = time.time()
        duration = end - start
        for k in prediction.keys(): evaluation[k] = copy.copy(prediction[k])         
        evaluation['text'] = text
        evaluation['duration'] = duration
        return evaluation

    def  predict_and_evaluate_intercept(self, actor, plan_params, human_eval, robot_eval, rel_dists):
        #actor - the human for which the plan is to be evaluated for
        #plan_params - parameters of the plan
        #human_path - predicted human path if the robot does nothing
        #robot_path - the planned path for the robot
        #rel_dists - the calculated distances along the two paths
        
        try:#search for index of dist < 0.5
            intercept_idx = next(idx for idx,rel_dist in enumerate(rel_dists) if rel_dist < 0.5)
        except:#if no intercept it will raise an exception and set the search return to None
            intercept_idx = None
            
        score = {}

        if intercept_idx <> None and not robot_eval['in_danger']:#if there is an intercept point in the path

#==============================================================================
#             print 'int point = ' + str(robot_eval['path'][intercept_idx]) + 'at idx ' + str(intercept_idx)
#             print 'robot path at speed ' + str(plan_params['speed'])
#             print robot_eval['path']
#             print 'human path'
#             print human_eval['path']
#==============================================================================

            #print 'dist'
            #print robot_eval['distances_along_path']
            #use the point where this first occurs as an intercept and ditch the rest of the human plan as it won't be executed
            #store the cropped path with actor name in a dict so it can be re-evaluated
            human_path = {}
            human_path['path'] = human_eval['path'][:intercept_idx+1]#updated path is up to and including intercept
            human_path['actor'] = human_eval['actor'] 
            human_path['goal'] = human_eval['goal']
            #evaluate intercepted path
            human_eval = self.evaluate_path(human_path)#recalculates the evaluation of the human path
            score['closest_danger'] = human_eval['closest_danger']#return the closest danger as that is what is being warned about and needs to be added to human knowledge if an ack is received
            #get distance walked to the intercept point from the results dictionary of the robot
            score['robot_walking_dist'] = robot_eval['distances_along_path'][intercept_idx]
            #get speed of robot
            score['robot_speed'] = plan_params['speed']
            #TODO improve evaluation to include probability of success based on likelihood the hunman will notice the robot and stop, that the robot is able to safely get to the target location etc.
            #components to be maximised are inverted
            score['danger_distance'] = -1 * human_eval['danger_distance']
            #all score components should be max of 1 or 2, so unless weights set differently will be roughly evenly weigthed in the final score
            score['robot_danger_dist'] = -1 * robot_eval['danger_dist']#min dist of robot to danger
            #calculate difference in times between human and robot arriving at intercept point, difference is number of time steps robot is waiting
            score['wait_time'] = -1 * (score['robot_speed']/0.25) * (intercept_idx - numpy.where(numpy.all(robot_eval['path']==robot_eval['path'][intercept_idx],axis=1))[0][0])

            #aim is to minimise this score
            score['total'] = self.settings['W_danger_distance']*score['danger_distance'] + \
                                self.settings['W_robot_walking_dist']*score['robot_walking_dist'] + \
                                self.settings['W_robot_speed']*score['robot_speed'] + \
                                self.settings['W_wait_time']* score['wait_time']                           
#==============================================================================
#             score['total'] =    self.settings['W_danger_distance']*score['danger_distance'] + \
#                                 (self.settings['W_robot_walking_dist']*score['robot_walking_dist'] * \
#                                 self.settings['W_robot_speed']*score['robot_speed']) + \
#                                 self.settings['W_wait_time']* score['wait_time']      
#==============================================================================
        else:
            #give a score that corresponds to a failed intercept
            score['closest_danger'] = None#return the closest danger as that is what is being warned about and needs to be added to human knowledge if an ack is received
            score['robot_walking_dist'] = 2
            score['robot_speed'] = 0.5
            score['danger_distance'] = 0
            score['robot_danger_dist'] = 0
            score['wait_time'] = 0
            score['total'] = self.settings['MAX_SCORE']#maximum score, to indicate a failed plan
            
        return score
        
    def predict_and_evaluate_warn(self, actor, plan_params, human_eval, robot_eval, rel_dists):
        #check for intercept and penalise in some way (no other scoring needed as plan failed)
        #get dist to human when warning given (currently when the robot stops moving) and use it to calculate if warning will succeed.
        #if success then update human world view and replan human path from place warned onwards
        ##get distance walked to the warning point from the results dictionary of the robot
        #get speed of robot
        #get human travel distance
        #get human proximity to dangers
        #get human proximity to goal at end? and time to get there? as above depends on if target is dangerous
        
        #calculate the distance of all the points in the trajectory from the goal
        #as the goal is on the human trajectory it will give the result of where to warn
        try:
            intercept_idx = next(idx for idx,rel_dist in enumerate(rel_dists) if rel_dist < 0.5)#search for index of dist < 0.5
        except:
            intercept_idx = None#if no intercept it will raise an exception and set the search return to None
            
        try:
            warn_idx = next(idx for idx,rel_dist in enumerate(rel_dists) if rel_dist < self.settings['hearing_dist'])#search for index of dist < 0.5
        except:
            warn_idx = None
        score = {}     
        
        #use stop_idx to calculate warn idx by looking at rel_dists for when robots are hearing dist appart
        
        #xan then use this value to calculate wait time
        #don't need to pass the path back as the robot controller just stops at the target point and then waits for the human to be in range before warnings
        
        score = {}
        if intercept_idx <> None or warn_idx == None or robot_eval['in_danger']:
            #give a score that corresponds to a failed warn because of an intercept or dangerous target
            score['closest_danger'] = None
            score['robot_walking_dist'] = 2
            score['robot_speed'] = 0.5
            score['danger_distance'] = 0
            score['robot_danger_dist'] = 0
            score['wait_time'] = 0
            score['total'] = self.settings['MAX_SCORE']#maximum score, to indicate a failed plan
        else:
            #assume warn successful if close enough,     
#==============================================================================
#             robot_goal = numpy.array(plan_params['position'])
#             print 'path ',robot_eval['path']
#             print 'pp ',robot_goal
#             print 'hp', human_eval['path']
#             print 'dists', rel_dists
# 
#==============================================================================
                
            warn_dist = numpy.min(rel_dists)#assume warning executed when robots closest together
            #TODO change simple threshold to calculate a success probabity for the warning based on the inter-robot distance when warning
            if warn_dist < self.settings['hearing_dist']:
            #so add the warned danger to a temp copy of the human's knowledge and re-predict the human path
                current_graph = self.__graphs[actor]
                current_graph_data = current_graph.get_data()#get a copy of the currently stored graph data
                #add the warned obstacle to it
                self.add_obstacles(human_eval['closest_danger'], actor)#assumes the warned danger is the one closest to the human
                #re-evaluate human path
                evaluation = self.predict_and_evaluate(actor, human_eval['goal'])#, plot, write_output)
                self.make_graph(actor, data=current_graph_data)#restore the graph
                self.__graphs[actor].plot_network(save=self.settings['session_path']+'plots')
                #there is no wait for ack in plan simulation
                score['closest_danger'] = evaluation['closest_danger']#return the closest danger as that is what is being warned about and needs to be added to human knowledge if an ack is received
                
                #return the actual plan goal - robot only walks to the warn location which is calculated in this method
                #score['goal'] = robot_eval['path'][stop_idx]
            
                #get distance walked to the intercept point from the results dictionary of the robot
                score['robot_walking_dist'] = robot_eval['distances_along_path'][warn_idx]
                #get speed of robot
                score['robot_speed'] = plan_params['speed']
                #calculate human distance to goal at intercept - utility of this depends somewhat on experiment set up, i.e., is goal a danger or is danger just on human path
                #TODO accumulate the components, weighted appropriately to form an evaluation score
                score['danger_distance'] =  -1 * evaluation['danger_distance']
                #all score components should be max of 1 or 2, so unless weights set differently will be roughlt evenly weigthed in the final score
                score['robot_danger_dist'] = 0#TODO update scoring for this plan type
                score['wait_time'] = -1 * (score['robot_speed']/0.25) * (warn_idx - numpy.where(numpy.all(robot_eval['path']==robot_eval['path'][warn_idx],axis=1))[0][0])
                score['total'] = self.settings['W_danger_distance']*score['danger_distance'] + \
                                    self.settings['W_robot_walking_dist']*score['robot_walking_dist'] + \
                                    self.settings['W_robot_speed']*score['robot_speed'] + \
                                    self.settings['W_wait_time']* score['wait_time'] 
                                    #self.settings['W_robot_danger_dist']* score['robot_danger_dist'] + \
            else:
                score['closest_danger'] = None
                score['robot_walking_dist'] = 2
                score['robot_speed'] = 0.5
                score['danger_distance'] = 0
                score['robot_danger_dist'] = 0
                score['wait_time'] = 0
                score['total'] = self.settings['MAX_SCORE']#maximum score, to indicate a failed plan
                
            
        return score
        
    def predict_and_evaluate_point(self, actor, plan_params, human_eval, robot_eval, rel_dists):
        #check for intercept and penalise in some way (no other scoring needed as plan failed)
        #get dist and angle to human when point given (currently when the robot stops moving) and use it to calculate if point will succeed.
        #if success then update human world view and replan human path from place warned onwards
        ##get distance walked to the point location from the results dictionary of the robot
        #get speed of robot
        #get human travel distance
        #get human proximity to dangers
        #get human proximity to goal at end? and time to get there? as above depends on if target is dangerous
        try:
            intercept_idx = next(idx for idx,rel_dist in enumerate(rel_dists) if rel_dist < 0.5)#search for index of dist < 0.5
        except:
            intercept_idx = None#if no intercept it will raise an exception and set the search return to None
        score = {}        

        intercept_idx = 1#debug

        if intercept_idx <> None:
            #give a score that corresponds to a failed point because of an intercept
            score['closest_danger'] = None
            score['robot_walking_dist'] = 2
            score['robot_speed'] = 0.5
            score['danger_distance'] = 0
            score['robot_danger_dist'] = 0
            score['wait_time'] = 0
            score['total'] = self.settings['MAX_SCORE']#maximum score, to indicate a failed plan
        else:
            #assume warn successful if close enough,     
            
            try:
                point_idx= numpy.where(numpy.all(robot_eval['path']==plan_params['position'],axis=1))[0][0]
            except:
                #print robot_eval['path']
                print 'point idx not found'
                point_idx = 0
            
            point_dist = rel_dists[point_idx]

            #TODO change simple threshold to calculate a success probabity for the warning based on the inter-robot distance when warning
            if point_dist < self.settings['pointing_dist']:
            #so add the warned danger to a temp copy of the human's knowledge and re-predict the human path
                current_graph = self.__graphs[actor]
                current_graph_data = current_graph.get_data()#get a copy of the currently stored graph data
                #add the warned obstacle to it
                self.add_obstacles(plan_params['point_pos'], actor)#assumes the warned danger is the one closest to the human
                #re-evaluate human path
                evaluation = self.predict_and_evaluate(actor, human_eval['goal'])#, plot, write_output)
                self.make_graph(actor, data=current_graph_data)#restore the graph
                self.__graphs[actor].plot_network(save=self.settings['session_path']+'plots')
                #there is no wait for ack in plan simulation
                score['closest_danger'] = evaluation['closest_danger']#return the closest danger as that is what is being warned about and needs to be added to human knowledge if an ack is received
            
                #get distance walked to the intercept point from the results dictionary of the robot
                score['robot_walking_dist'] = robot_eval['distances_along_path'][-1]
                #get speed of robot
                score['robot_speed'] = plan_params['speed']
                #calculate human distance to goal at intercept - utility of this depends somewhat on experiment set up, i.e., is goal a danger or is danger just on human path
                #TODO accumulate the components, weighted appropriately to form an evaluation score
                score['danger_distance'] = MAX_DIST - evaluation['danger_distance']
                #all score components should be max of 1 or 2, so unless weights set differently will be roughlt evenly weigthed in the final score
                score['robot_danger_dist'] = 0#TODO update scoring for this plan type
                score['wait_time'] = 0
                score['total'] = self.settings['W_danger_distance']*score['danger_distance'] + \
                                    self.settings['W_robot_walking_dist']*score['robot_walking_dist'] + \
                                    self.settings['W_robot_speed']*score['robot_speed'] + \
                                    self.settings['W_robot_danger_dist']* score['robot_danger_dist'] + \
                                    self.settings['W_wait_time']* score['wait_time'] 
            else:
                score['closest_danger'] = None#return the closest danger as that is what is being warned about and needs to be added to human knowledge if an ack is received
                score['robot_walking_dist'] = 2
                score['robot_speed'] = 0.5
                score['danger_distance'] = 0
                score['robot_danger_dist'] = 0
                score['wait_time'] = 0
                score['total'] = self.settings['MAX_SCORE']#maximum score, to indicate a failed plan
                
            
        return score


    def predict_dists(self, robot_path, actor_path):
        #path lists should be the same length due to scaling of resolution to ensure this but run a check just in case and pad the end of shorter path with end location
                    
        result = []
        #for each element in both lists
        r_path = numpy.asmatrix(robot_path)
        a_path = numpy.asmatrix(actor_path)
        #calc and store distance between robot and actor
        delta = r_path - a_path
        #result.append((numpy.sum((robot_path[idx] - actor_path[idx])**2))**0.5)
        result = numpy.linalg.norm(delta, axis=1)
        result = numpy.asarray(result)
        result = result.flatten()
        #return the list of distances            
        return result

    def predict_all(self, actor, plan_params, Robot_Plan=False, plot=False):
        #plan_params is the dictionary of parameters for all 3 plan types as suggested by the GP
        start = time.time()
        if isinstance(plot,basestring):
            plot_cs=plot.replace('XXX', actor)+actor
        else:
            plot_cs = plot

        if 'DEBUG_goal_HUMAN_A' in self.settings:
            current_situation = self.predict_and_evaluate(actor, goal=self.settings['DEBUG_goal_HUMAN_A'], plot=plot_cs)
        else:
            current_situation = self.predict_and_evaluate(actor, plot=plot_cs)#use the current plan graph to assess no plan situation        
        
        no_action = 'No intervention:' + actor + ' Goal ' + str(current_situation['goal']) + ' Danger ' + str(current_situation['in_danger']) + ' C_Danger ' + str(current_situation['closest_danger']) + ' d_dist ' + str(current_situation['danger_distance'])
        self.__logger.write(no_action)        
        
        #if the position is specified as ['actor', proportion of path, proportion of distance]
        if isinstance(plan_params['position'][0], basestring):
            #robot goal must be calulated from a percentage along a human path
            robot_goal_idx = int(len(current_situation['path'] * plan_params['position'][1]))
            robot_goal = current_situation['path'][robot_goal_idx]
        else:
            robot_goal=plan_params['position']
        if isinstance(plot,basestring):
            plot_rp = plot.replace('XXX', 'ROBOT')+'ROBOT'    
        else:
            plot_rp = plot

        if Robot_Plan:
            robot_plan = Robot_Plan
        else:
            robot_plan = self.predict_path(self.__self_name, goal=robot_goal, plot=plot_rp)#g.find_path(start, plan_params['position'], plot)
        #print self.__plan
        #print robot_goal
        #print robot_plan['path']
        #print robot_plan['distances_along_path']
        #print current_situation['path']
        #if the plan param for target position is specified as  ['actor', proportion of path, proportion of distance]
        if isinstance(plan_params['position'][0], basestring):
            robot_goal_idx = int(len(robot_plan['path'] * plan_params['position'][2]))#clip path length by multiplying the total length by a proportion stored in the plan_params
            robot_plan['path'] = robot_plan['path'][:robot_goal_idx+1]
            robot_plan['distances_along_path'] = robot_plan['distances_along_path'][:robot_goal_idx+1]
        
            
        #if the path lists are different lengths then extend the shorter path and distance lists with duplicates of the stationary position,
        if len(robot_plan['path']) > len(current_situation['path']):
            for _ in range(len(robot_plan['path']) - len(current_situation['path'])):
                current_situation['path'] = numpy.vstack([current_situation['path'],current_situation['path'][-1]])
                current_situation['distances_along_path'] = numpy.append(current_situation['distances_along_path'], current_situation['distances_along_path'][-1])
        elif len(robot_plan['path']) < len(current_situation['path']):
            for _ in range(len(current_situation['path']) - len(robot_plan['path'])):
                robot_plan['path'] = numpy.vstack([robot_plan['path'],robot_plan['path'][-1]])
                robot_plan['distances_along_path'] = numpy.hstack([robot_plan['distances_along_path'],robot_plan['distances_along_path'][-1]])
        
                #add goal to end of trajectory as if the goal was too close to danger it won.t be on there and it needs to be checked to establish plan validity
        robot_path_comp = numpy.vstack((robot_plan['path'], numpy.array(robot_goal)))
        robot_danger_dists = Utilities.distance2points(current_situation['closest_danger'],robot_path_comp)
        robot_danger_dists = numpy.array(robot_danger_dists)
        robot_plan['danger_dist'] = numpy.min(robot_danger_dists)
        if  robot_plan['danger_dist'] <  self.__safe_distance: 
            robot_plan['in_danger'] = True   
        else:
            robot_plan['in_danger'] = False

        #pass both planned paths to a function that calculates distances between the robots at each time step
        #the graphs for human and robot are scaled for each plan so that one node in the graph is the distance travelled by that robot in one time step
        robot_actor_dists = self.predict_dists(robot_plan['path'], current_situation['path'])
        #pass the returned distances to evaluation functions (a different one for each plan type) to evaluate the plan
        if self.__plan == 'move':
            result = self.predict_and_evaluate_intercept(actor, plan_params, current_situation, robot_plan, robot_actor_dists)
            #print 'move'
        elif self.__plan == 'warn':
            result = self.predict_and_evaluate_warn(actor, plan_params, current_situation, robot_plan, robot_actor_dists)
            #print 'warn'
        elif self.__plan == 'point':
            result = self.predict_and_evaluate_point(actor, plan_params, current_situation, robot_plan, robot_actor_dists)
            #print 'point'
        if result:    
            score = actor + ' goal ' + str(robot_goal) + ' speed ' + str(plan_params['speed']) + ' CD ' + str(result['closest_danger']) + ' WD ' + str(result['robot_walking_dist']) + ' S ' + str(result['robot_speed']) + ' DD ' + str(result['danger_distance'])+ ' WT ' + str(result['wait_time']) + ' RDD ' + str(result['robot_danger_dist']) +' Total ' + str(result['total'])
        else:
            score = str(result)
        self.__logger.write(score)
       
        

#==============================================================================
#         if plot: pyplot.savefig(template.replace('XXX', 'current'))
#         closest_danger = current_situation['closest_danger']
#         intercept_position = self.get_intercept_position(current_situation, plot=plot)
#         if plot: pyplot.savefig(template.replace('XXX', 'intercept_position'))
#         result_call = self.predict_and_evaluate_call_out(actor, closest_danger, plot=plot)
#         if plot: pyplot.savefig(template.replace('XXX', 'call_action'))
#         result_intercept = self.predict_and_evaluate_intercept(actor, intercept_position, plot=plot)
#         if plot: pyplot.savefig(template.replace('XXX', 'intercept_action'))
# 
#         danger_current = current_situation['in_danger']
#         danger_call = result_call['in_danger']
#         danger_intercept = result_intercept['in_danger']
# 
#         call_action = closest_danger
#         intercept_action = intercept_position['intercept_position']
# 
#         do_call = False
#         do_intercept = False
#         if danger_current and not danger_call: do_call = True
#         if danger_current and not danger_intercept: do_intercept = True
#==============================================================================
        end = time.time()
        duration = end - start
        results = {}
        results['path'] = robot_plan['path']
        results['distances_along_path'] = robot_plan['distances_along_path']
        results['current'] = current_situation
        results['duration'] = duration
        results['score'] = result
        results['log_msg'] = score
        results['inter_rob_dists'] = robot_actor_dists#return the list of inter-robot dists so whether to warn can be evaluated by the controller
        try:
            results['goal'] = result['goal']#if the plan is warn or point the predict or evaluate function returns an updated goal
        except:
            pass
        return results



