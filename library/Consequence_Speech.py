import matplotlib.pyplot as pyplot
import Utilities
import PathPlanning
import copy
import Rotate
import numpy
import time
import FrozenObject

Utilities.set_numpy_style()


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


class ConsequenceEngine(FrozenObject.FrozenClass):
    def __init__(self, self_name, actor_names, tracker, engine_name='CEngine'):
        self.__logger = Utilities.Logger(engine_name)
        self.__logger.write('Creating consequence engine ' + engine_name + ' for ' + self_name)

        self.__safe_distance = 0.30
        self.__speed_threshold = 0.0

        self.__tracker = tracker
        self.__self_name = self_name
        self.__engine_name = engine_name
        self.__actor_names = actor_names
        self.__graphs = {}

        self.__dangers = []
        self.__danger_locations = None

        for actor in actor_names: self.make_graph(actor)
        self.make_graph(self_name)

        self._freeze()

    def set_speed_threshold(self, threshold):
        self.__speed_threshold = threshold

    def make_graph(self, actor, data=False, step=None):
        graph = PathPlanning.Planner(self.__tracker, data=data, step=step)
        self.__graphs[actor] = copy.copy(graph)

    def motion_command(self, goal, plot=False):
        g = self.__graphs[self.__self_name]
        start = self.__tracker.get_position(self.__self_name)[0:2]
        result = g.motion_command(start, goal, plot)
        return result

    def set_dangers(self, dangers):
        if type(dangers) is not list: dangers = [dangers]
        locations = []
        for danger in dangers:
            danger = self.__tracker.get_position(danger)
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
        for a in self.__actor_names: objects.remove(a)  # actors can not be goals
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
        start = self.__tracker.get_position(actor)
        g = self.__graphs[actor]
        result = g.find_path(start, goal, plot)
        result['actor'] = actor
        result['goal'] = goal
        path = result['path']
        distances_along_path = numpy.diff(path, n=1, axis=0)
        distances_along_path = numpy.vstack((numpy.array([0, 0]), distances_along_path))
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
        if type(plot) == str: pyplot.savefig(plot)
        return result

    def evaluate_path(self, result):
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
        min_danger = self.__dangers[min_index]
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
        return evaluation

    def predict_and_evaluate(self, actor, goal=None, plot=False, write_output=True):
        start = time.time()
        prediction = self.predict_path(actor, goal, plot)
        evaluation = self.evaluate_path(prediction)
        text = 'RESULT P&E: ' + prediction['actor'] + ', '
        text += str(prediction['goal']) + ', '
        text += evaluation['closest_danger'] + ', '
        text += Utilities.num2str(evaluation['danger_distance']) + ', '
        text += str(evaluation['in_danger']) + ', '
        if write_output: self.__logger.write(text)
        end = time.time()
        duration = end - start
        for k in prediction.keys(): evaluation[k] = copy.copy(prediction[k])
        evaluation['text'] = text
        evaluation['duration'] = duration
        return evaluation

    def predict_and_evaluate_call_out(self, actor, pointed_out_obstacle, plot=False, write_output=True):
        goal=None
        current_graph = self.__graphs[actor]
        current_graph_data = current_graph.get_data()
        self.add_obstacles(pointed_out_obstacle, actor)
        if write_output: self.__logger.write('Call out action tested: ' + str(pointed_out_obstacle))
        evaluation = self.predict_and_evaluate(actor, goal, plot, write_output)
        if plot: pyplot.title('Prediction call-out action')
        self.make_graph(actor, data=current_graph_data)
        return evaluation

    def predict_and_evaluate_intercept(self, actor, intercept_result, plot=False, write_output=True):
        if write_output: self.__logger.write('interception action tested')
        goal = intercept_result['final_actor_position']
        evaluation = self.predict_and_evaluate(actor, goal, plot, write_output)
        if plot: pyplot.title('Prediction intercept action')
        return evaluation

    def get_intercept_position(self, prediction, plot=False):
        start = time.time()
        path = prediction['path']
        distances_along_path = prediction['distances_along_path']
        path_length = path.shape[0]
        intercept_position = None
        for index in range(0, path_length):
            current_position = path[index, :]
            current_prediction = self.predict_path(self.__self_name, current_position, False)
            current_path = current_prediction['path']
            self_travel_distance = numpy.max(current_prediction['distances_along_path'])
            actor_travel_distance = distances_along_path[index]
            if actor_travel_distance > 1.1 * self_travel_distance:
                intercept_position = current_path[-1, :]
                break

        if intercept_position is None: intercept_position = self.__tracker.get_position(self.__self_name)[0:2]
        final_distances = Utilities.distance2points(intercept_position, path)
        too_close = numpy.where(final_distances < 0.3)[0]
        too_close_index = path.shape[0] - 1
        if too_close.size > 0:too_close_index = numpy.min(too_close)
        final_actor_position = path[too_close_index, :]

        if plot:
            actor = prediction['actor']
            actor_current_position = self.__tracker.get_position(actor)
            self_position = self.__tracker.get_position(self.__self_name)
            g = self.__graphs[self.__self_name]
            dangers = self.__danger_locations
            g.plot_network()
            pyplot.hold(True)
            pyplot.plot(path[:, 0], path[:, 1], 'b', linewidth=2)
            pyplot.scatter(actor_current_position[0], actor_current_position[1], s=100, c='b', zorder=100)
            pyplot.scatter(final_actor_position[0], final_actor_position[1], s=100, c='b', alpha=0.5, zorder=100)
            pyplot.scatter(self_position[0], self_position[1], s=100, c='r', zorder=101)
            pyplot.scatter(intercept_position[0], intercept_position[1], s=100, c='y', alpha=1.00, zorder=102)
            pyplot.scatter(current_position[0], current_position[1], s=200, c='y', alpha=0.5, zorder=102)
            pyplot.scatter(dangers[:, 0], dangers[:, 1], s=500, c='red', alpha=0.5, zorder=10)
            pyplot.hold(False)
            ax = pyplot.gca()
            ax.set_aspect(1)
            if type(plot) == str: pyplot.savefig(plot)

        end = time.time()
        duration = end - start
        result = {}
        result['duration'] = duration
        result['intercept_position'] = intercept_position # the position that can be actually reached using the path planning
        result['intercept_goal'] = current_position # the position on the actors path selected for interception
        result['final_distances'] = final_distances
        result['final_actor_position'] = final_actor_position
        return result

    def predict_all(self, actor, plot=False, template=None):
        start = time.time()
        current_situation = self.predict_and_evaluate(actor, plot=plot)
        if plot: pyplot.savefig(template.replace('XXX', 'current'))
        closest_danger = current_situation['closest_danger']
        intercept_position = self.get_intercept_position(current_situation, plot=plot)
        if plot: pyplot.savefig(template.replace('XXX', 'intercept_position'))
        result_call = self.predict_and_evaluate_call_out(actor, closest_danger, plot=plot)
        if plot: pyplot.savefig(template.replace('XXX', 'call_action'))
        result_intercept = self.predict_and_evaluate_intercept(actor, intercept_position, plot=plot)
        if plot: pyplot.savefig(template.replace('XXX', 'intercept_action'))

        danger_current = current_situation['in_danger']
        danger_call = result_call['in_danger']
        danger_intercept = result_intercept['in_danger']

        call_action = closest_danger
        intercept_action = intercept_position['intercept_position']

        do_call = False
        do_intercept = False
        if danger_current and not danger_call: do_call = True
        if danger_current and not danger_intercept: do_intercept = True
        end = time.time()
        duration = end - start
        results = {}
        results['current'] = current_situation
        results['call'] = result_call
        results['intercept'] = result_intercept
        results['do_call'] = do_call
        results['do_intercept'] = do_intercept
        results['call_action'] = call_action
        results['intercept_action'] = intercept_action
        results['duration'] = duration
        return results



