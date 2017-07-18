__author__ = 'dieter'

__author__ = 'dieter'

import time
import numpy
import matplotlib.pyplot as pyplot
import scipy.io as io
import HTMLlog
import Utilities
import Rotate
Utilities.set_numpy_style()


class ConsequenceEngine:
    def __init__(self, self_name, actor_names, tracked_objects, tracker):

        if type(actor_names) != list: actor_names = [actor_names]
        if type(tracked_objects) != list: actor_names = [tracked_objects]

        self.logger = Utilities.Logger('CENGINE')
        self.tracker = tracker
        self.self_name = self_name
        self.agent_names = actor_names
        self.description = 'Objects: ' + str(tracked_objects) + ', actor: ' + str(actor_names)
        self.logger.write('Creating consequence engine for ' + self_name + '. ' + self.description)
        self.tracked_objects = tracked_objects
        self.speed = 1.5

        # results of the steps
        self.data = {}
        self.infer_goal_results = []
        self.predict_path_results = []
        self.target_results = []
        self.simulation_results = []
        self.evaluation_results = []
        self.interrupt_results = []

        # interrupt history
        self.interrupt_history = []

        # plot place holder
        self.figure = pyplot.figure('Consequence_Engine', figsize=(20, 10))
        self.axis1 = self.figure.add_subplot(121)
        self.axis2 = self.figure.add_subplot(122)
        self.agent_colors = ['#B54B44', '#5571F2', '#00A1CB', '#00A1CB']
        self.marker_colors = ['#D0D102', '#D70060', '#00A1CB']
        self.bg_color = '#F0EDE5'

    def get_state(self):
        self.data['infer_goal_results'] = self.infer_goal_results
        self.data['predict_path_results'] = self.predict_path_results
        self.data['target_results'] = self.target_results
        self.data['simulation_results'] = self.simulation_results
        self.data['evaluation_results'] = self.evaluation_results
        self.data['interrupt_history'] = self.interrupt_history
        self.data['interrupt_results'] = self.interrupt_results
        self.data['position_' + self.self_name] = self.get_object_data([self.self_name], 'position')[0]
        self.data['velocity_' + self.self_name] = self.get_object_data([self.self_name], 'velocity')[0]
        for name in self.agent_names:
            self.data['position_' + name] = self.get_object_data([name], 'position')[0]
            self.data['velocity_' + name] = self.get_object_data([name], 'velocity')[0]
        object_positions, object_names = self.get_object_data()
        self.data['objects'] = object_names
        self.data['object_positions'] = object_positions
        return self.data

    def save_state(self, file_name):
        self.get_state()
        io.savemat(file_name, self.data)

    def status2html(self, header='status', file_name='consequence_status.html'):
        state = self.get_state()
        code = HTMLlog.dict2table(state, title=header)
        f = open(file_name, 'a')
        f.write(code)
        f.close()

    def update_interrupt_history(self, value):
        self.interrupt_history.append(value)

    def get_tracker_data(self, name, value):
        data = self.tracker.enquire(name)
        if value.startswith('v'): return data['velocity']
        if value.startswith('p'): return data['position']
        if value.startswith('r'): return data['rotation']
        if value.startswith('s'): return data['speed']

    def get_agent_data(self, agent_name, value):
        if type(agent_name) == int: agent_name = self.agent_names[agent_name]
        data = self.get_tracker_data(agent_name, value)
        return data

    def get_self_data(self, value):
        name = self.self_name
        data = self.get_tracker_data(name, value)
        return data

    def get_object_data(self, objects=None, value='position'):
        if objects is None: objects = self.tracked_objects[:]
        if type(objects) != list: objects = [objects]
        all_data = []
        for name in objects:
            data = self.get_tracker_data(name, value)
            all_data.append(data)
        all_data = numpy.array(all_data)
        return all_data, objects

    def merge_targets(self, raw_results):
        all_targets = []
        all_names = []
        for key, value in raw_results.iteritems():
            if Utilities.is_array(value): value = value.tolist()
            if len(value) > 0:
                if type(value[0]) != list:  value = [value]
            for index in range(0, len(value)):
                if len(value) > 1: name = key + str(index)
                if len(value) == 1: name = key
                all_names.append(name)
            all_targets = all_targets + value
        all_targets = numpy.array(all_targets)
        return all_targets, all_names

    ##################################

    def world2agent(self, agent_name, world_x, world_y):
        rotation_agent = self.get_agent_data(agent_name, 'rotation')
        robot_position = self.get_agent_data(agent_name, 'position')
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

    def infer_agent_goal(self, agent_name):
        if type(agent_name) == int: agent_name = self.agent_names[agent_name]
        angles = []
        objects = self.tracked_objects[:]
        objects.remove(agent_name)

        position_agent = self.get_agent_data(agent_name, 'position')
        velocity_agent = self.get_agent_data(agent_name, 'velocity')

        speed_agent = numpy.linalg.norm(velocity_agent)
        for object_name in objects:
            object_position, _ = self.get_object_data(object_name)
            world_x = object_position[0][0]
            world_y = object_position[0][1]
            result = self.world2agent(agent_name, world_x, world_y)
            print object_name, result
            angle = abs(result[2])
            angles.append(angle)

        angles = numpy.array(angles)
        index = numpy.argmin(angles)
        inferred_goal = objects[index]
        if speed_agent < 0.01: inferred_goal = agent_name

        velocity_agent_norm = Utilities.normalize(velocity_agent) * 0.25
        velocity_agent_norm = velocity_agent_norm + position_agent
        velocity_agent_plot = numpy.vstack((position_agent, velocity_agent_norm))

        self.logger.write('Inferred Goal: ' + str(inferred_goal))

        result = {}
        result['inferred_goal'] = inferred_goal
        result['velocity_agent_norm'] = velocity_agent_norm
        result['velocity_agent_plot'] = velocity_agent_plot
        result['position_agent'] = position_agent
        result['velocity_agent'] = velocity_agent
        result['speed_agent'] = speed_agent
        result['angles'] = angles
        result['objects'] = objects

        return result

    def predict_agent_path(self, agent):
        assert self.infer_goal_results is not [], "Goal has to be inferred first."
        prediction = self.infer_goal_results[agent]
        start = self.get_agent_data(agent, 'position')
        target, _ = self.get_object_data(prediction['inferred_goal'], 'position')
        time_i = numpy.linspace(0, 1, 25)
        f = Utilities.interpolate_path(start, target, 0, 1)
        path = f(time_i)

        results = {}
        results['start'] = start
        results['target'] = target
        results['path'] = path
        return results

    def path_target_positions(self, agent):
        assert self.predict_path_results is not [], "Path has to be predicted first."
        path = self.predict_path_results[agent]
        start = path['start']
        target = path['target']
        time_i = numpy.linspace(0, 1, 5)
        f = Utilities.interpolate_path(start, target, 0, 1)
        positions = f(time_i)
        positions = positions[1:-1, :]
        # remove positions too close to agent
        distance_from_agent = Utilities.distance2points(start, positions)
        far_enough = distance_from_agent > 0.5
        positions = positions[far_enough, :]
        return positions

    #############################

    def infer_goals(self):
        agents = self.agent_names[:]
        result = {}
        for agent in agents:
            self.logger.write('Inferring goal for agent ' + str(agent))
            goal_result = self.infer_agent_goal(agent)
            result[agent] = goal_result
        self.infer_goal_results = result

    def predict_paths(self):
        agents = self.agent_names[:]
        result = {}
        for agent in agents:
            self.logger.write('Predicting path for agent ' + str(agent))
            predict_result = self.predict_agent_path(agent)
            result[agent] = predict_result
        self.predict_path_results = result

    def generate_targets(self, add_self=False):
        agents = self.agent_names[:]
        result = {}
        # path positions
        for agent in agents:
            self.logger.write('Generating path target positions for agent ' + str(agent))
            path_targets = self.path_target_positions(agent)
            result[agent] = path_targets
        # object positions
        positions, names = self.get_object_data(value='position')
        names.remove(self.self_name)
        for i in range(0, len(names)):
            current_name = names[i]
            if current_name not in self.agent_names: result[current_name] = positions[i, :]

        # self position
        if add_self:
            self_position, _ = self.get_object_data(self.self_name, 'position')
            result[self.self_name] = self_position

        # merge everything into a single array and list of names
        all_targets, all_names = self.merge_targets(result)
        new_result = {}
        new_result['targets'] = all_targets
        new_result['names'] = all_names
        new_result['raw'] = result
        self.target_results = new_result

    def get_agent_speeds(self, normalize=False):
        speeds = []
        for agent_name in self.agent_names:
            speed = self.get_agent_data(agent_name, 'speed')
            self.logger.write('Speed Agent ' + agent_name + ': ' + str(speed))
            speeds.append(speed)
        speeds = numpy.array(speeds)
        max_value = numpy.max(numpy.abs(speeds))
        if normalize: speeds = speeds/max_value
        return speeds

    def simulate_target(self, target_index):
        assert self.target_results is not [], "Generate target positions first."
        # make robot interpolate function
        robot_target_positions = self.target_results['targets']
        robot_target_position_names = self.target_results['names']
        self_target_position = robot_target_positions[int(target_index), :]
        self_target_position += (numpy.random.rand(3) * 0.01)
        self_current_position, _ = self.get_object_data(self.self_name, 'position')
        f_self = Utilities.interpolate_path(self_current_position, self_target_position, 0, 1)
        self_travel_distance = numpy.linalg.norm(self_current_position - self_target_position)

        final_agent_positions = []
        agent_names = [self.self_name]
        agent_targets = [self_target_position.flatten()]
        agent_positions = [self_current_position.flatten()]
        agent_functions = [f_self]
        agent_horizons = [1]

        norm_speeds = self.get_agent_speeds(True)
        self.logger.write('Normalized Speeds: ' + str(norm_speeds))

        for agent_name in self.agent_names:
            agent_index = self.agent_names.index(agent_name)
            norm_speed = norm_speeds[agent_index]
            agent_current_position, _ = self.get_object_data(agent_name, 'position')
            agent_target_position = self.predict_path_results[agent_name]['target']
            agent_target_position += (numpy.random.rand(3) * 0.01)
            agent_travel_distance = numpy.linalg.norm(agent_current_position - agent_target_position)
            agent_travel_duration = (agent_travel_distance / self_travel_distance) * self.speed / norm_speed
            f_agent = Utilities.interpolate_path(agent_current_position, agent_target_position, 0, agent_travel_duration)
            agent_functions.append(f_agent)
            agent_targets.append(agent_target_position.flatten())
            agent_positions.append(agent_current_position.flatten())
            agent_names.append(agent_name)
            agent_horizons.append(agent_travel_duration)

        horizon = max(agent_horizons)
        delta_t = 0.1
        times = Utilities.arange(0, horizon, delta_t)
        indices = range(0, len(agent_functions))

        for focus_index in indices:
            self.logger.write('Starting simulation for ' + agent_names[focus_index])
            focus_function = agent_functions[focus_index]
            other_indices = indices[:]
            other_indices.remove(focus_index)
            predicted_final_time = horizon
            predicted_final_position = agent_targets[focus_index]

            for other_index in other_indices:
                self.logger.write('..Interaction partner: ' + agent_names[other_index])
                other_function = agent_functions[other_index]
                for time_step in times:
                    final_focus = focus_function(time_step)
                    final_other = other_function(time_step)
                    distance = numpy.linalg.norm(final_focus - final_other)
                    if distance < 0.3: break
                if time_step < predicted_final_time:
                    predicted_final_time = time_step
                    predicted_final_position = final_focus
            final_agent_positions.append(predicted_final_position.flatten())

        result = {}
        result['agent_targets'] = agent_targets
        result['agent_positions'] = agent_positions
        result['final_agent_positions'] = final_agent_positions
        result['agent_names'] = agent_names
        result['agent_horizons'] = agent_horizons
        result['robot_target_positions'] = robot_target_positions
        result['robot_target_position_names'] = robot_target_position_names
        result['robot_target_position'] = self_target_position
        return result

    def plot_simulation(self, target_name):
        assert self.target_results is not [], "Run simulations first."
        target_names = self.simulation_results['simulation_names']
        if type(target_name) == int: target_name = target_names[target_name]
        data = self.simulation_results[target_name]
        agent_names = data['agent_names']
        agent_positions = data['agent_positions']
        agent_targets = data['agent_targets']
        final_agent_positions = data['final_agent_positions']
        targets = data['robot_target_positions']

        nr_agents = len(self.agent_names) + 1
        nr_robot_targets = len(targets)
        indices = range(0, nr_agents)

        self.axis1.clear()
        # plot agent paths
        for index in indices:
            n = agent_names[index]
            p = agent_positions[index][0:2]
            t = agent_targets[index][0:2]
            f = final_agent_positions[index][0:2]

            self.axis1.hold(True)
            self.axis1.annotate('',
                                xy=t, xycoords='data', xytext=p, textcoords='data',
                                arrowprops=dict(arrowstyle="->",
                                                linewidth=3,
                                                alpha=0.5,
                                                color=self.agent_colors[index],
                                                patchB=None,
                                                shrinkB=0,
                                                connectionstyle="arc3,rad=0",
                                                ),
                                )

            self.axis1.annotate('',
                                xy=f, xycoords='data', xytext=p, textcoords='data',
                                arrowprops=dict(arrowstyle="->",
                                                linewidth=3,
                                                color=self.agent_colors[index],
                                                patchB=None,
                                                shrinkB=0,
                                                connectionstyle="arc3,rad=0",
                                                ),
                                )

        # plot objects
        object_positions, object_names = self.get_object_data()
        self.axis1.scatter(object_positions[:, 0], object_positions[:, 1], s=50, marker='o', c=self.marker_colors[0])

        # plot robot targets
        for index in range(0, nr_robot_targets):
            self.axis1.scatter(targets[index, 0], targets[index, 1], s=25, marker='o', c=self.marker_colors[1],
                               alpha=0.5)
            self.axis1.text(targets[index, 0] + 0.05, targets[index, 1] + 0.05, target_names[index])

        self.axis1.set_xlim((-1.5, 1.5))
        self.axis1.set_ylim((-1.5, 1.5))
        self.axis1.set_title(target_name)
        self.axis1.hold(False)

    def simulate_all_targets(self):
        start = time.time()
        assert self.target_results is not [], "Generate target positions first."
        targets = self.target_results['targets']
        names = self.target_results['names']
        number_targets = targets.shape[0]
        result = {}
        for index in range(0, number_targets):
            partial_result = self.simulate_target(index)
            result[names[index]] = partial_result
        end = time.time()
        duration = end - start
        result['simulation_duration'] = duration
        result['simulation_names'] = names
        self.simulation_results = result
        return duration

    def evaluate_simulations(self, dangerous_objects=None, command_issued=False, acceptable_level=0.75):
        results = {}
        assert self.simulation_results is not [], "Simulate targets first."

        agent_names = self.agent_names[:]
        agent_indices = range(1, len(agent_names) + 1)

        simulation_names = self.simulation_results['simulation_names'][:]
        dangerous_positions, _ = self.get_object_data(dangerous_objects, 'position')

        results['dangerous_positions'] = dangerous_positions
        results['dangerous_objects'] = dangerous_objects
        results['simulation_names'] = simulation_names

        action_values = []

        for key in simulation_names:
            values = []
            distances = []
            data = self.simulation_results[key]
            final_positions = data['final_agent_positions']
            # get own distance to dangerous objects
            final_self_position = final_positions[0]
            distances_self = Utilities.distance2points(final_self_position, dangerous_positions)
            min_distance_self = numpy.min(distances_self)
            value_self = Utilities.sigmoid(min_distance_self, threshold=1, slope=5)
            values.append(value_self)
            distances.append(min_distance_self)
            # get agents' distances to dangerous objects
            for index in agent_indices:
                final_agent_position = final_positions[index]
                distances_agent = Utilities.distance2points(final_agent_position, dangerous_positions)
                min_distance_agent = numpy.min(distances_agent)
                value_agent = Utilities.sigmoid(min_distance_agent, threshold=1, slope=5)
                values.append(value_agent)
                distances.append(min_distance_agent)
            # weigh values
            min_value_agent = min(values[1:])
            if not command_issued and min_value_agent >= acceptable_level:
                self.logger.write('Ethical Case 1: no command, human not in danger')
                action_value = sum(values[1:]) + value_self
            if not command_issued and min_value_agent < acceptable_level:
                self.logger.write('Ethical Case 2: no command, human is in danger')
                action_value = sum(values[1:])
            if command_issued:
                self.logger.write('Ethical Case 3: command issued')
                action_value = sum(values[1:])

            action_values.append(action_value)
            partial_result = {}
            partial_result['final_distances'] = distances
            partial_result['values'] = values
            results[key] = partial_result

        results['action_values'] = numpy.array(action_values)
        self.evaluation_results = results

    def plot_evaluation_results(self):
        assert self.evaluation_results is not [], "Evaluate simulations first."
        names = self.evaluation_results['simulation_names']
        values = self.evaluation_results['action_values']
        x_values = range(0, len(names))
        self.axis2.clear()
        self.axis2.hold(True)
        self.axis2.bar(x_values, values)
        self.axis2.set_xticks(x_values)
        self.axis2.set_xticklabels(names, rotation=45)
        self.axis2.hold(False)
        self.axis2.set_title('Simulation values')
        self.axis2.set_xlim((0, len(names)))
        self.axis2.set_ylim((0, 4))

    def interrupt(self, threshold=0.2):
        assert self.evaluation_results is not [], "Evaluation of simulation results should be done first."
        values = self.evaluation_results['action_values']
        simulation_names = self.evaluation_results['simulation_names'][:]

        min_value = numpy.min(values)
        max_value = numpy.max(values)
        #suitable = values > (max_value - 0.1)
        #suitable_indices = numpy.where(suitable)[0]
        #max_index = numpy.min(suitable_indices)

        max_index = numpy.argmax(values)

        target_name = simulation_names[max_index]
        target_position = self.simulation_results[target_name]['robot_target_position']

        interrupt = False
        difference = max_value - min_value
        if difference > threshold: interrupt = True
        self.update_interrupt_history(difference)

        result = {}
        result['values'] = values
        result['actions'] = simulation_names
        result['difference'] = difference
        result['target_position'] = target_position
        result['max_index'] = max_index
        result['target_name'] = target_name
        result['interrupt'] = interrupt
        self.interrupt_results = result

        return interrupt, target_position, target_name




  # def infer_agent_goal(self, agent_name):
  #       if type(agent_name) == int: agent_name = self.agent_names[agent_name]
  #       position_agent = self.get_agent_data(agent_name, 'position')
  #       velocity_agent = self.get_agent_data(agent_name, 'velocity')
  #       rotation_agent = self.get_agent_data(agent_name, 'rotation')
  #
  #       speed_agent = numpy.linalg.norm(velocity_agent)
  #       angles = []
  #       objects = self.tracked_objects[:]
  #
  #       for object_name in objects:
  #           object_position, _ = self.get_object_data(object_name)
  #           object_vector = object_position - position_agent
  #           object_vector = Utilities.normalize(object_vector)
  #           angle = Utilities.angle_vectors(object_vector, velocity_agent)
  #           angles.append(angle)
  #
  #       angles = numpy.array(angles)
  #       index = numpy.argmin(angles)
  #       inferred_goal = objects[index]
  #       if speed_agent < 0.01: inferred_goal = agent_name
  #
  #       velocity_agent_norm = Utilities.normalize(velocity_agent) * 0.25
  #       velocity_agent_norm = velocity_agent_norm + position_agent
  #       velocity_agent_plot = numpy.vstack((position_agent, velocity_agent_norm))
  #
  #       result = {}
  #       result['inferred_goal'] = inferred_goal
  #       result['velocity_agent_norm'] = velocity_agent_norm
  #       result['velocity_agent_plot'] = velocity_agent_plot
  #       result['position_agent'] = position_agent
  #       result['velocity_agent'] = velocity_agent
  #       result['speed_agent'] = speed_agent
  #       result['angles'] = angles
  #       result['objects'] = objects
  #
  #       return result