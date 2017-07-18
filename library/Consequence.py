__author__ = 'dieter'

import time
import HTML
import numpy
import matplotlib.pyplot as pyplot
import scipy.io as io
import Natsort
import Utilities


Utilities.set_numpy_style()


class ConsequenceEngine:
    def __init__(self, self_name, actor_name, objects, tracker):
        self.logger = Utilities.Logger('CENGINE')
        self.tracker = tracker
        self.self_name = self_name
        self.agent_name = actor_name
        self.description = 'Objects: ' + str(objects) + ', actor: ' + str(actor_name)
        self.logger.write('Creating consequence engine for ' + self_name + '. ' + self.description)
        self.objects = objects
        self.speed = 1.5

        # results of the steps
        self.data = {}
        self.infer_goal_results = None
        self.predict_path_results = None
        self.generated_target_positions = None
        self.simulation_results = None
        self.evaluation_results = None
        self.interrupt_results = None

        # interrupt history
        self.interrupt_history = numpy.ones(40) * numpy.nan
        self.raw_interrupt_history = []

        # plot
        self.figure = pyplot.figure('Consequence_Engine', figsize=(25, 10))
        self.axis1 = self.figure.add_subplot(231)
        self.axis2 = self.figure.add_subplot(232)
        self.axis3 = self.figure.add_subplot(233)
        self.axis4 = self.figure.add_subplot(234)
        self.axis5 = self.figure.add_subplot(235)
        self.axis6 = self.figure.add_subplot(236)

    def save_state(self, file_name):
        self.data['infer_goal_results'] = self.infer_goal_results
        self.data['predict_path_results'] = self.predict_path_results
        self.data['generated_target_positions'] = self.generated_target_positions
        self.data['simulation_results'] = self.simulation_results
        self.data['evaluation_results'] = self.evaluation_results
        self.data['interrupt_history'] = self.interrupt_history
        self.data['raw_interrupt_history'] = self.raw_interrupt_history
        self.data['interrupt_results'] = self.interrupt_results
        io.savemat(file_name, self.data)

    def status2html(self, file_name='consequence_status.html'):
        f = open(file_name, 'w')
        for field in range(0,5):
            if field == 0:
                data = self.infer_goal_results
                title = 'Infer goal'
            if field == 1:
                data = self.predict_path_results
                title = 'Predict path'
            if field == 2:
                data = self.generated_target_positions
                title = 'Generate Positions'
            if field == 3:
                data = self.simulation_results
                title = 'Simulation results'
            if field == 4:
                data = self.evaluation_results
                title = 'Evaluation Results'

            f.write('<h1>' + title + '</h1>')
            keys = data.keys()
            table = HTML.Table(header_row=['Key', 'Data'])
            for key in keys:
                line = [str(key), str(data[key])]
                row = HTML.TableRow(line)
                table.rows.append(row)
            html_code = str(table)
            f.write(html_code)
        f.close()

    def update_interrupt_history(self, value):
        self.raw_interrupt_history.append(value)
        nans = numpy.where(numpy.isnan(self.interrupt_history))[0]
        if nans.shape[0] > 0:
            first = numpy.min(nans)
            self.interrupt_history[first] = value
        if nans.shape[0] == 0:
            self.interrupt_history = numpy.roll(self.interrupt_history, -1)
            self.interrupt_history[-1] = value

    def get_objects_positions(self, objects=None):
        if objects is None: objects = self.objects[:]
        positions = []
        for name in objects:
            position = self.get_object_position(name)
            positions.append(position)
        positions = numpy.array(positions)
        return positions, objects

    def get_dangerous_objects(self):
        dangerous_names = self.objects[:]
        dangerous_names = [i for i in dangerous_names if i.startswith('DANGER')]
        (dangerous_positions, dangerous_names) = self.get_objects_positions(dangerous_names)
        return dangerous_positions, dangerous_names

    def get_agent_data(self, value):
        data = self.tracker.enquire(self.agent_name)
        if value.startswith('v'): return data['velocity']
        if value.startswith('p'): return data['position']

    def get_self_data(self, value):
        data = self.tracker.enquire(self.self_name)
        if value.startswith('v'): return data['velocity']
        if value.startswith('p'): return data['position']

    def get_object_position(self, object_name):
        data = self.tracker.enquire(object_name)
        return data['position']

    def infer_goal(self, do_plot=False):
        position_agent = self.get_agent_data('position')
        velocity_agent = self.get_agent_data('velocity')
        speed_agent = numpy.linalg.norm(velocity_agent)
        angles = []
        for object_name in self.objects:
            object_position = self.get_object_position(object_name)
            object_vector = object_position - position_agent
            object_vector = Utilities.normalize(object_vector)
            angle = Utilities.angle_vectors(object_vector, velocity_agent)
            angles.append(angle)

        angles = numpy.array(angles)
        index = numpy.argmin(angles)
        inferred_goal = self.objects[index]  #
        if speed_agent < 0.01: inferred_goal = self.agent_name

        results = {}
        results['angles'] = angles
        results['objects'] = self.objects
        results['speed_agent'] = speed_agent
        results['inferred_goal'] = inferred_goal
        results['velocity_agent'] = velocity_agent
        self.infer_goal_results = results

        if do_plot:
            velocity_agent_norm = Utilities.normalize(velocity_agent) * 0.25
            velocity_agent_norm = velocity_agent_norm + position_agent
            velocity_agent_plot = numpy.vstack((position_agent, velocity_agent_norm))
            self.axis1.clear()
            self.axis1.hold(True)
            self.axis1.scatter(position_agent[0], position_agent[1], s=100, marker='s', c='red')
            self.axis1.plot(velocity_agent_plot[:, 0], velocity_agent_plot[:, 1], 'r-')
            for object_name in self.objects:
                object_position = self.get_object_position(object_name)
                color = '0.5'
                if object_name == inferred_goal: color = 'red'
                self.axis1.scatter(object_position[0], object_position[1], s=100, marker='p', c=color, alpha=0.5)
                self.axis1.text(object_position[0] + 0.05, object_position[1] + 0.05, object_name[0])
            self.axis1.hold(False)
            self.axis1.set_title('Goal inference')
            self.axis1.set_xlim((-1.5, 1.5))
            self.axis1.set_ylim((-1.5, 1.5))

            self.data['velocity_agent_norm'] = velocity_agent_norm
            self.data['velocity_agent_plot'] = velocity_agent_plot
            self.data['position_agent'] = position_agent
            self.data['velocity_agent'] = velocity_agent
            self.data['speed_agent'] = speed_agent

    def predict_path(self, do_plot=False):
        assert self.infer_goal_results is not None, "Goal has to be inferred first."
        prediction = self.infer_goal_results
        start = self.get_agent_data('position')
        target = self.get_object_position(prediction['inferred_goal'])
        time_i = numpy.linspace(0, 1, 25)
        f = Utilities.interpolate_path(start, target, 0, 1)
        path = f(time_i)

        results = {}
        results['start'] = start
        results['target'] = target
        results['path'] = path
        self.predict_path_results = results

        if do_plot:
            self.axis2.clear()
            self.axis2.hold(True)
            self.axis2.scatter(start[0], start[1], s=100, marker='s', c='red')
            self.axis2.scatter(target[0], target[1], s=100, marker='p', c='red')
            self.axis2.plot(path[:, 0], path[:, 1], linewidth=2, c='black', alpha=0.25)
            self.axis2.hold(False)
            self.axis2.set_title('Path prediction')
            self.axis2.set_xlim((-1.5, 1.5))
            self.axis2.set_ylim((-1.5, 1.5))

    def generate_target_positions(self, manual=None, number=5, do_plot=False):
        assert self.predict_path_results is not None, "Path has to be predicted first."
        path = self.predict_path_results
        start = path['start']
        target = path['target']
        time_i = numpy.linspace(0, 1, number + 2)
        f = Utilities.interpolate_path(start, target, 0, 1)
        positions = f(time_i)
        positions = positions[1:-1, :]

        #remove positions too close to agent
        distance_from_agent = Utilities.distance2points(start, positions)
        far_enough = distance_from_agent > 0.5
        positions = positions[far_enough, :]
        number_of_positions = positions.shape[0]
        position_names = ['None'] * number_of_positions

        if manual is not None:
            manual_names = manual[:]
            manual_names.reverse() #reverse list order - as the positions are added to the front in reveresed order
            position_names = manual_names + position_names
            for x in manual:
                position = self.get_object_position(x)
                positions = numpy.vstack((position, positions))

        number_of_positions = positions.shape[0]

        #add names of positions that are not in manual
        for index in range(0, number_of_positions):
            txt = position_names[index]
            if txt == 'None': txt = 'P' + str(index)
            position_names[index] = txt

        results = {}
        results['number'] = number
        results['number_of_positions'] = number_of_positions
        results['manual'] = manual
        results['positions'] = positions
        results['position_names'] = position_names
        self.generated_target_positions = results

        if do_plot:
            self.axis3.clear()
            self.axis3.hold(True)
            self.axis3.scatter(start[0], start[1], s=100, marker='s', c='red')
            self.axis3.scatter(target[0], target[1], s=100, marker='p', c='red')
            self.axis3.scatter(positions[:, 0], positions[:, 1], s=100, marker='.', c='black', alpha=0.5)
            for index in range(0, number_of_positions):
                txt = position_names[index]
                self.axis3.text(positions[index, 0] + 0.05, positions[index, 1] + 0.05, txt)
            self.axis3.hold(False)
            self.axis3.set_title('Target Positions')
            self.axis3.set_xlim((-1.5, 1.5))
            self.axis3.set_ylim((-1.5, 1.5))

    def simulate_target(self, index, do_plot=False):
        assert self.generated_target_positions is not None, "Generate target positions first."

        agent_position = self.get_agent_data('position')
        self_position = self.get_self_data('position')

        positions = self.generated_target_positions['positions']
        self_target = positions[int(index), :]
        self_target += (numpy.random.rand(3) * 0.01)

        agent_target = self.predict_path_results['target']
        agent_target += (numpy.random.rand(3) * 0.01)

        self_distance = numpy.linalg.norm(self_position - self_target)
        agent_distance = numpy.linalg.norm(agent_position - agent_target)

        agent_duration = (agent_distance / self_distance) * self.speed

        f_self = Utilities.interpolate_path(self_position, self_target, 0, 1)
        f_agent = Utilities.interpolate_path(agent_position, agent_target, 0, agent_duration)

        horizon = max([1, agent_duration])
        delta_t = 0.1

        times = Utilities.arange(0, horizon, delta_t)

        for time_step in times:
            final_self = f_self(time_step)
            final_agent = f_agent(time_step)
            distance = numpy.linalg.norm(final_agent - final_self)
            if distance < 0.3: break

        results = {}
        results['self_distance'] = self_distance
        results['agent_distance'] = agent_distance
        results['agent_position'] = agent_position
        results['agent_target'] = agent_target
        results['self_position'] = self_position
        results['self_target'] = self_target
        results['final_agent_position'] = final_agent
        results['final_self_position'] = final_self
        results['agent_duration'] = agent_duration
        results['horizon'] = horizon

        self.data['self_position'] = self_position

        if do_plot:
            agent_position = self.get_agent_data('position')
            self_position = self.get_self_data('position')

            self.axis4.clear()
            self.axis4.hold(True)
            self.axis4.scatter(self_position[0], self_position[1], s=100, marker='s', c='green')
            self.axis4.scatter(final_self[0], final_self[1], s=100, marker='*', c='green')
            self.axis4.scatter(self_target[0], self_target[1], s=150, marker='o', c='green', alpha=0.25)

            self.axis4.scatter(agent_position[0], agent_position[1], s=100, marker='s', c='red')
            self.axis4.scatter(final_agent[0], final_agent[1], s=100, marker='*', c='red')
            self.axis4.scatter(agent_target[0], agent_target[1], s=150, marker='o', c='red', alpha=0.25)

            self.axis4.hold(False)
            self.axis4.set_title('Simulate position ' + str(int(index)))
            self.axis4.set_xlim((-1.5, 1.5))
            self.axis4.set_ylim((-1.5, 1.5))

        return results

    def simulate_all_targets(self):
        start = time.time()
        assert self.generated_target_positions is not None, "Generate target positions first."
        positions = self.generated_target_positions['positions']
        number_targets = positions.shape[0]
        result = {}
        for index in range(0, number_targets):
            partial_result = self.simulate_target(index)
            key = 'position' + str(index).zfill(3)
            result[key] = partial_result
        end = time.time()
        duration = end - start
        result['simulation_duration'] = duration
        self.simulation_results = result
        return duration

    def evaluate_simulations(self, dangerous_objects=None, command_issued=False, acceptable_level=0.75, do_plot=False):
        results = {}
        assert self.simulation_results is not None, "Simulate targets first."
        keys = self.simulation_results.keys()
        position_keys = [i for i in keys if i.startswith('position')]
        position_keys = Natsort.natsorted(position_keys)

        if dangerous_objects is not None:
            (dangerous_positions, dangerous_names) = self.get_objects_positions(dangerous_objects)

        if dangerous_objects is None:
            (dangerous_positions, dangerous_names) = self.get_dangerous_objects()

        results['dangerous_positions'] = dangerous_positions
        results['dangerous_names'] = dangerous_names

        values = []
        for position in position_keys:
            simulation_result = self.simulation_results[position]

            # get agent's distance to dangerous objects
            final_agent_position = simulation_result['final_agent_position']
            distances_agent = Utilities.distance2points(final_agent_position, dangerous_positions)
            min_distance_agent = numpy.min(distances_agent)

            # get own distance to dangerous objects
            final_self_position = simulation_result['final_self_position']
            distances_self = Utilities.distance2points(final_self_position, dangerous_positions)
            min_distance_self = numpy.min(distances_self)

            # calculate values
            value_agent = Utilities.sigmoid(min_distance_agent, threshold=0.25, slope=10)
            value_self = Utilities.sigmoid(min_distance_self, threshold=0.25, slope=10)

            # weigh values
            if not command_issued and value_agent >= acceptable_level:
                self.logger.write('Ethical Case 1: no command, human not in danger')
                value = value_agent + value_self
            if not command_issued and value_agent < acceptable_level:
                self.logger.write('Ethical Case 2: no command, human is in danger')
                value = value_agent
            if command_issued:
                self.logger.write('Ethical Case 3: command issued')
                value = value_agent

            values.append(value)

            partial_result = {}
            partial_result['distance_agent'] = distances_agent
            partial_result['distance_self'] = distances_self
            partial_result['min_distance_self'] = min_distance_self
            partial_result['min_distance_agent'] = min_distance_agent
            partial_result['value_agent'] = value_agent
            partial_result['value_self'] = value_self
            partial_result['value'] = value
            results[position] = partial_result

        values = numpy.array(values)
        results['values'] = values
        self.evaluation_results = results

        if do_plot:
            position_names = self.generated_target_positions['position_names']
            x_values = range(0, len(position_keys))
            self.axis5.clear()
            self.axis5.hold(True)
            self.axis5.bar(x_values, values)
            self.axis5.set_xticks(x_values)
            self.axis5.set_xticklabels(position_names, rotation=45)
            self.axis5.hold(False)
            self.axis5.set_title('Simulation values')
            self.axis5.set_xlim((0, len(position_keys)))
            self.axis5.set_ylim((0, 2.5))

    def interrupt(self, threshold=0.2, print_message=False, do_plot=False):
        assert self.evaluation_results is not None, "Evaluation of simulation results should be done first."
        values = self.evaluation_results['values']
        positions = self.generated_target_positions['positions']
        position_names = self.generated_target_positions['position_names']
        interrupt = False
        min_value = numpy.min(values)
        max_value = numpy.max(values)

        suitable = values > (max_value - 0.1)
        suitable_indices = numpy.where(suitable)[0]
        max_index = numpy.min(suitable_indices)

        target = positions[max_index, :]
        target_name = position_names[max_index]
        difference = max_value - min_value
        if difference > threshold: interrupt = True
        self.update_interrupt_history(difference)

        if do_plot:
            number = self.interrupt_history.shape[0]
            self.axis6.clear()
            self.axis6.hold(True)
            self.axis6.plot(self.interrupt_history, linewidth=3, c='red', alpha=0.5)
            self.axis6.plot([1, number], [threshold, threshold], c='black')
            self.axis6.set_xlim((0, number))
            self.axis6.set_ylim((0, 2))
            self.axis6.set_title('Interrupt history')
            self.axis6.hold(False)

        str_x = Utilities.num2str(target[0], 2)
        str_y = Utilities.num2str(target[1], 2)
        message = 'Interrupt ' + str(interrupt) + '. Best target: ' + str(max_index) + ' @ ' + str_x + ' ' + str_y

        result = {}
        result['interrupt'] = interrupt
        result['max_index'] = max_index
        result['target'] = target
        result['target_name'] = target_name
        self.interrupt_results = result

        if print_message: self.logger.write(message)
        return interrupt, max_index, target, target_name



#def evaluate_simulations(self, dangerous_objects=None, do_plot=False):
#        assert self.simulation_results is not None, "Simulate targets first."
#        keys = self.simulation_results.keys()
#        position_keys = [i for i in keys if i.startswith('position')]
#        position_keys = Natsort.natsorted(position_keys)
#
#        if dangerous_objects is not None:
#            (dangerous_positions, dangerous_names) = self.get_objects_positions(dangerous_objects)
#
#        if dangerous_objects is None:
#            (dangerous_positions, dangerous_names) = self.get_dangerous_objects()
#
#        results = {}
#        results['dangerous_positions'] = dangerous_positions
#        results['dangerous_names'] = dangerous_names
#
#        values = []
#        for position in position_keys:
#            simulation_result = self.simulation_results[position]
#
#            # get agent's distance to dangerous objects
#            final_agent_position = simulation_result['final_agent_position']
#            distances_agent = Utilities.distance2points(final_agent_position, dangerous_positions)
#            min_distance_agent = numpy.min(distances_agent)
#
#            # get own distance to dangerous objects
#            final_self_position = simulation_result['final_self_position']
#            distances_self = Utilities.distance2points(final_self_position, dangerous_positions)
#            min_distance_self = numpy.min(distances_self)
#
#            # get distance between target and current agent position
#            # agent_position = simulation_result['agent_position'][0:2]
#            # target_position = simulation_result['self_target'][0:2]
#            # target_agent_distance = numpy.linalg.norm(agent_position - target_position)
#
#            # calculate values
#            value_agent = Utilities.sigmoid(min_distance_agent, threshold=0.25, slope=10)
#            value_self = Utilities.sigmoid(min_distance_self, threshold=0.25, slope=10)
#
#            # weigh values
#            value = value_agent + value_self
#            values.append(value)
#
#            partial_result = {}
#            partial_result['distance_agent'] = distances_agent
#            partial_result['distance_self'] = distances_self
#            partial_result['min_distance_self'] = min_distance_self
#            partial_result['min_distance_agent'] = min_distance_agent
#            partial_result['value_agent'] = value_agent
#            partial_result['value_self'] = value_self
#            partial_result['value'] = value
#            results[position] = partial_result
#
#        values = numpy.array(values)
#        results['values'] = values
#        self.evaluation_results = results
#
#        if do_plot:
#            x_values = range(0, len(position_keys))
#            self.axis5.clear()
#            self.axis5.hold(True)
#            self.axis5.bar(x_values, values)
#            self.axis5.set_xticks(x_values)
#            self.axis5.set_xticklabels(position_keys, rotation=45)
#            self.axis5.hold(False)
#            self.axis5.set_title('Simulation values')
#            self.axis5.set_xlim((0, len(position_keys)))
#            self.axis5.set_ylim((0, 2.5))