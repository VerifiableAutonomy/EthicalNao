import networkx
import numpy
import copy
import time
import math
import Utilities
import warnings
import FrozenObject
import matplotlib.pyplot as pyplot
from scipy.signal import savgol_filter

Utilities.set_numpy_style()


def test_path_consistency(path):
    differential = (numpy.sum((numpy.diff(path, axis=0)**2), axis=1))**0.5
    if differential.size == 0: return
    min_differential = numpy.min(differential)
    if min_differential == 0: warnings.warn('Path contains duplicate nodes!')


def smooth_path(path):
    path_x = path[:, 0]
    path_y = path[:, 1]
    smooth_window = 3
    if numpy.size(path_x)-2 > 2:
        mid_x = savgol_filter(path_x[1:-1], smooth_window, 1)
        mid_y = savgol_filter(path_y[1:-1], smooth_window, 1)
        path_x = numpy.hstack((path_x[0], mid_x, path_x[-1]))
        path_y = numpy.hstack((path_y[0], mid_y, path_y[-1]))
    path = numpy.vstack((path_x, path_y))
    path = numpy.transpose(path)
    return path


class Planner(FrozenObject.FrozenClass):
    def __init__(self, tracker=None, x_range=None, y_range=None, step=None, data=False):
        if x_range is None: x_range = [-2, 2]
        if y_range is None: y_range = [-2, 2]
        if step is None: step = 0.25

        self.__step = step
        self.__x_range = x_range
        self.__y_range = y_range

        self.__graph = None
        self.__positions = None
        self.__positions_array = None
        self.__tracker = tracker

        self.__obstacle_range = 0.5
        self.__patch_range = 0.25

        # these props are set when updating/making the graph
        self.__obstacles = []
        self.__patches = []
        self.__nodes = []
        self.__resolution = None

        self.make_default_graph()
        if data: self.set_data(data)

        #self._freeze()

    def get_data(self):
        result = {}
        result['step'] = copy.copy(self.__step)
        result['x_range'] = copy.copy(self.__x_range)
        result['y_range'] = copy.copy(self.__y_range)
        result['obstacles'] = copy.copy(self.__obstacles)
        result['patches'] = copy.copy(self.__patches)
        result['obstacle_range'] = copy.copy(self.__obstacle_range)
        result['patch_range'] = copy.copy(self.__patch_range)
        result = copy.copy(result)
        return result

    def get_nodes(self):
        return self.__positions

    def get_graph(self):
        return self.__graph

    def set_data(self, data):
        data = copy.copy(data)
        self.__x_range = copy.copy(data['x_range'])
        self.__y_range = copy.copy(data['y_range'])
        self.__step = copy.copy(data['step'])
        self.__obstacles = copy.copy(data['obstacles'])
        self.__patches = copy.copy(data['patches'])
        self.__obstacle_range = copy.copy(data['obstacle_range'])
        self.__patch_range = copy.copy(data['patch_range'])
        self.update_graph()
        
    def set_step(self, step):
        self.__step = step
        self.update_graph()
        
    def get_step(self):
        return self.__step

    def make_default_graph(self):
        positions = {}
        positions_array = []
        nodes = []
        x = numpy.arange(self.__x_range[0], self.__x_range[1] + self.__step, self.__step)
        y = numpy.arange(self.__y_range[0], self.__y_range[1] + self.__step, self.__step)
        n1 = numpy.size(x)
        n2 = numpy.size(y)
        # make node positions
        for i in range(0, n1):
            for j in range(0, n2):
                cds = numpy.array([x[i], y[j]])
                positions[(i, j)] = cds
                nodes.append((i, j))
        graph = networkx.grid_2d_graph(n1, n2)
        # add diagonal connections
        for i in range(0, n1 - 1):
            for j in range(0, n2):
                if (j - 1) >= 0: graph.add_edge((i, j), (i + 1, j - 1), weight= math.sqrt(2))
                if (j + 1) <= (n2 - 1): graph.add_edge((i, j), (i + 1, j + 1), weight= math.sqrt(2))
        # get positions in same order as the nodes are stored
        for x in nodes: positions_array.append(positions[x])
        self.__graph = graph
        self.__positions = positions
        self.__positions_array = numpy.array(positions_array)
        self.__nodes = nodes
        self.__resolution = ((numpy.sqrt(2) * self.__step) / 2)
        return graph, positions

    def test_graph_consistency(self, verbose_level=0):
        n = len(self.__nodes)
        consistent = True
        for i in range(0, n):
            node = self.__nodes[i]
            position = self.__positions[node]
            position_array = self.__positions_array[i, :]
            if not numpy.allclose(position, position_array): consistent = False
            if verbose_level > 1: print i, node, position, position_array
        if verbose_level > 0:
            print 'Number of nodes:', n
            print 'Number of positions:', len(self.__positions)
            print 'Size of array positions:', self.__positions_array.shape
            print 'Consistent:', consistent
        return consistent

    def plot_network(self, reuse_graph=False, save=False):
        if not reuse_graph: handle = pyplot.figure()
        networkx.draw_networkx(self.__graph, self.__positions, node_color=[0.5, 0.5, 0.5], node_size=50, with_labels=False, edge_color='grey')
        pyplot.hold(True)
        for obstacle in self.__obstacles:
            if isinstance(obstacle, str): obstacle = self.__tracker.get_position(obstacle)[0:2]
            pyplot.scatter(obstacle[0], obstacle[1], marker='^', s=150, c='red', zorder=15, alpha=1)
        for patch in self.__patches:
            if isinstance(patch, str): patch = self.__tracker.get_position(patch)[0:2]
            pyplot.scatter(patch[0], patch[1], marker='v', s=150, c='green', zorder=16, alpha=1)
        pyplot.hold(False)
        ax = pyplot.gca()
        ax.set_aspect(1)
        pyplot.xlim(Utilities.minmax(self.__positions_array[:, 0]))
        pyplot.ylim(Utilities.minmax(self.__positions_array[:, 1]))
        if save: pyplot.savefig(save)
        #if not reuse_graph: 
        return handle
        

    def remove_node(self, node_key):
        if not type(node_key) is tuple: node_key = self.__graph.nodes()[node_key]
        node_index = self.__nodes.index(node_key)
        self.__graph.remove_node(node_key)
        self.__positions.pop(node_key)
        self.__nodes.remove(node_key)
        self.__positions_array = numpy.delete(self.__positions_array, node_index, 0)

    def get_positions(self, position_type):
        if position_type.startswith('o'): landmarks = self.__obstacles[:]
        if position_type.startswith('p'): landmarks = self.__patches[:]
        positions = []
        for landmark in landmarks:
            if isinstance(landmark, str): landmark = self.__tracker.get_position(landmark)[0:2]
            landmark = landmark[0:2]
            positions.append(landmark)
        positions = numpy.array(positions)
        positions = numpy.reshape(positions,(-1, 2))
        return positions

    def distance_from_landmarks(self, point, position_type):
        if isinstance(point, str): point = self.__tracker.get_position(point)[0:2]
        landmark_positions = self.get_positions(position_type)
        if landmark_positions.size == 0: return 999
        distances = Utilities.distance2points(point, landmark_positions)
        min_distance = numpy.min(distances)
        return min_distance

    def update_graph(self):
        # This function updates the internal representation of the graph. This should normally only be called internally.
        self.make_default_graph()
        remove_list = []
        for node in self.__graph.nodes():
            node_position = self.__positions[node]
            min_obst = self.distance_from_landmarks(node_position, 'obstacles')
            min_pat = self.distance_from_landmarks(node_position, 'patches')
            if min_obst < self.__obstacle_range and min_pat > self.__patch_range: remove_list.append(node)
        for x in remove_list: self.remove_node(x)
        consistent = self.test_graph_consistency()
        assert consistent, "Graph not consistent!"

    def set_obstacles(self, obstacles):
        if type(obstacles) is not list: obstacles = [obstacles]
        self.__obstacles = obstacles
        self.make_default_graph()
        self.update_graph()

    def get_obstacles(self):
        return self.__obstacles

    def add_obstacles(self, obstacles):
        if type(obstacles) is not list: obstacles = [obstacles]
        current = self.__obstacles
        for x in obstacles: current.append(x)
        self.set_obstacles(current)

    def set_patches(self, patches):
        self.__patches = patches
        self.make_default_graph()
        self.update_graph()

    def add_patches(self, patches):
        if type(patches) is not list: patches = [patches]
        current = self.__patches
        for x in patches: current.append(x)
        self.set_patches(current)

    def find_closest_node(self, position, origin=None):
        if origin is None: origin = position
        if isinstance(position, str): position = self.__tracker.get_position(position)[0:2]
        if isinstance(origin, str): origin = self.__tracker.get_position(origin)[0:2]
        node_positions = self.__positions_array
        distance_to_origin = Utilities.distance2points(origin, node_positions)
        distance_to_position = Utilities.distance2points(position, node_positions)
        distance_threshold = numpy.min(distance_to_position) + self.__resolution

        too_far = numpy.where(distance_to_position > distance_threshold)
        distance_to_origin[too_far] = 999
        index = numpy.argmin(distance_to_origin)
        node = self.__nodes[index]
        min_distance = distance_to_position[index]
        return node, min_distance

    def plot_path(self, result, save_file=False):
        path = result['path']
        # goal = result['goal']
        start = result['start']
        direction = result['direction']
        vector = numpy.vstack((start, direction))
        fig = self.plot_network()
        pyplot.hold(True)
        pyplot.scatter(path[:, 0], path[:, 1], s=60, c='orange', zorder=20)
        pyplot.plot(vector[:, 0], vector[:, 1], linewidth=3, zorder=21, alpha=0.5)
        pyplot.scatter(start[0], start[1], marker='s', s=60, c='black', zorder=18, alpha=0.5)
        # if goal: pyplot.scatter(goal[0], goal[1], marker='s', s=60, c='black', zorder=53, alpha=0.5)
        pyplot.hold(False)        
        if type(save_file) == str: pyplot.savefig(save_file+'_path.png')
        pyplot.close(fig)

    def find_path(self, start, goal, plot=False):
        start_time = time.time()
        if isinstance(start, str): start = self.__tracker.get_position(start)[0:2]
        if isinstance(goal, str): goal = self.__tracker.get_position(goal)[0:2]
        start = numpy.array(start[0:2])
        goal = numpy.array(goal[0:2])

        start_node, _ = self.find_closest_node(start, goal)
        start_node_position = self.__positions[start_node]
        goal_node, _ = self.find_closest_node(goal, start_node_position)

        try:
            planning_start = time.time()
            node_path = networkx.shortest_path(self.__graph, start_node, goal_node, weight='weight')
            planning_end = time.time()
            planning_duration = planning_end - planning_start
            success = True
        except networkx.NetworkXNoPath:
            warnings.warn('Path planning failed')
            node_path = []
            success = False
            planning_duration = None

        path = numpy.empty([0, 2])
        for x in node_path: path = numpy.vstack((path, self.__positions[x]))

        # if goal_node <> start_node ...
        if path.shape[0] > 2:
            # remove first node if trivially close to start
            dist_to_start = numpy.sum((path[0, :] - start)**2)
            if dist_to_start < self.__step/20: path = path[1:, :]
            # remove last node if trivially close to goal
            dist_to_goal = numpy.sum((path[-1, :] - goal)**2)
            if dist_to_goal < self.__step/20: path = path[0:-1, :]

        # add goal position - if not too far from existing nodes (i.e. too close to obstacles)
        obst_dist = self.distance_from_landmarks(goal, 'obs')
        patch_dist = self.distance_from_landmarks(goal, 'pat')
        if obst_dist > self.__obstacle_range or patch_dist <= self.__patch_range: path = numpy.vstack((path, goal))

        test_path_consistency(path)

        n = int(math.ceil(1.0/self.__step))
        direction = numpy.mean(path[0:n, :], axis=0)

        end_time = time.time()
        duration = end_time - start_time

        result = {}
        result['goal'] = goal
        result['start'] = start
        result['duration'] = duration
        result['planning_duration'] = planning_duration
        result['success'] = success
        result['nodes'] = node_path
        result['path'] = path
        result['network'] = self.__graph
        result['positions'] = self.__positions
        result['direction'] = direction
        result = copy.copy(result)
        if plot: self.plot_path(result, plot)
        return result

    def find_path_via(self, waypoints, plot=False):
        start_time = time.time()
        success = True
        parts = {}
        complete_path = numpy.empty([0, 2])
        for index in range(1, len(waypoints)):
            start = waypoints[index - 1]
            goal = waypoints[index]
            part = self.find_path(start, goal)
            if not part['success']: success = False
            partial_path = part['path']
            #if index < len(waypoints) - 1: partial_path = partial_path[:-1, ]
            complete_path = numpy.vstack((complete_path, partial_path))
            parts['part' + str(index)] = part

        test_path_consistency(complete_path)

        n = int(math.ceil(1.0/self.__step))
        direction = numpy.mean(complete_path[0:n, :], axis=0)
        end_time = time.time()

        differential = (numpy.sum((numpy.diff(complete_path, axis=0)**2), axis=1))**0.5
        min_differential = numpy.min(differential)
        if min_differential == 0: warnings.warn('Identical nodes in planned path.')

        duration = end_time - start_time

        result = {}
        result['duration'] = duration
        result['direction'] = direction
        result['start'] = waypoints[0]
        result['goal'] = goal
        result['success'] = success
        result['parts'] = parts
        result['path'] = complete_path
        if plot: self.plot_path(result, plot)
        return result

    def motion_command(self, current_position, goal, plot=False):
        if isinstance(current_position, str): current_position = self.__tracker.get_position(current_position)[0:2]
        if isinstance(goal, str): goal = self.__tracker.get_position(goal)[0:2]
        current_position = numpy.array(current_position)
        path = self.find_path(current_position, goal, plot)
        direction_vector = path['direction']
        # distance_to_direction_vector = numpy.sum((current_position - direction_vector) ** 2) ** 0.5
        distance_to_goal = numpy.sum((current_position - goal) ** 2) ** 0.5
        next_position = numpy.copy(direction_vector)
        # if distance_to_direction_vector < self.__resolution: next_position = numpy.copy(goal)
        vector_length = (numpy.sum(next_position**2))**0.5

        command = {}
        command['vector_length'] = vector_length
        command['next_position'] = next_position
        command['distance'] = distance_to_goal
        command['path'] = path
        return command

        # N = network()

        # obstacle = numpy.array([[0, 0], [1, 1],[-0.9,0]])
        # N.add_obstacles(obstacle, 0.5)

        # start = numpy.array([-1, -1.5])
        # goal = numpy.array([-0.5, 1])
        # N.find_path(start, goal, True)

        # pyplot.show()


    # def update_graph(self):
    #     self.make_default_graph()
    #     for obstacle in self.obstacles:
    #         if isinstance(obstacle, str): obstacle = self.tracker.get_position(obstacle)[0:2]
    #         obstacle = obstacle[0:2]
    #         remove_list = []
    #         for node in self.graph.nodes():
    #             node_position = self.positions[node]
    #             distance_to_obstacle = numpy.sqrt(numpy.sum((node_position - obstacle) ** 2))
    #             distance_to_patches = 100000
    #             for patch in self.patches:
    #                 if isinstance(patch, str): patch = self.tracker.get_position(patch)[0:2]
    #                 distance_to_patch = numpy.sqrt(numpy.sum((node_position - patch) ** 2))
    #                 if distance_to_patch < distance_to_patches: distance_to_patches = distance_to_patch
    #             if distance_to_obstacle < self.obstacle_range and distance_to_patches > self.patch_range:
    #                 remove_list.append(node)
    #         for x in remove_list: self.remove_node(x)


        # def find_closest_node(self, position, origin):
        # if isinstance(position, str): position = self.__tracker.get_position(position)[0:2]
        # if isinstance(origin, str): origin = self.__tracker.get_position(origin)[0:2]
        # data = []
        # for node in self.__graph.nodes():
        #     node_position = self.__positions[node]
        #     distance_to_position = numpy.sqrt(numpy.sum((node_position - position) ** 2)) ** 0.5
        #     distance_to_origin = numpy.sqrt(numpy.sum((node_position - origin) ** 2)) ** 0.5
        #     data.append([node, distance_to_position, distance_to_origin])
        # data = numpy.array(data)
        # # this is done to avoid selecting the node that happens to be closest to a goal due to the grid discritization
        # data = data[data[:, 2].argsort()]
        # distance_threshold = numpy.min(data[:, 1]) + self.__resolution
        # selected = data[:, 1] <= distance_threshold
        # data = data[selected, :]
        # current_node = data[0, 0]
        # return current_node

        #dist_nodes = Utilities.distance2points(goal, self.__positions_array)
        #dist_nodes = numpy.min(dist_nodes)
        #dist_goal = self.distance_from_landmarks(goal, 'obs')
        #if dist_goal > self.__obstacle_range and dist_nodes > self.__resolution: path = numpy.vstack((path, goal))
