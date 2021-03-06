import sys

import numpy

sys.path.append('pynaoqi-python2.7-2.1.2.17-linux64')

from naoqi import ALProxy
import time
import threading
import Rotate
import Utilities
import math
import matplotlib.pyplot as pyplot
import scipy.io as io
import scipy.spatial as spatial


def sign(x):
    if x < 0: return -1
    if x > 0: return 1
    return 0


def num2str(number):
    stg = '{0:.' + str(2) + 'f}'
    return stg.format(number)


class Robot:
    def __init__(self, address, robot_name, tracker):
        self.logger = Utilities.Logger(robot_name)
        self.logger.write('Instantiating robot with ip ' + address)
        self.speed_factor = 1

        self.ip_address = address
        self.robot_name = robot_name
        self.position = None
        self.velocity = None
        self.rotation = None
        self.current_angle = None
        self.current_x = None
        self.current_y = None

        self.desired_x = None
        self.desired_y = None

        self.original_target = None
        self.corrected_target = None
        self.corrected_path = None

        self.motion_counter = 0
        self.safety_distance = 0.5
        self.arrival_threshold = 0.1
        self.sway_threshold = 0.7

        # define arena and make a grid of alternative targets
        # self.arena_points = numpy.array([[-1.2, 0.75], [1.5, 0.75], [1.5, -1.7, ], [-1.2, -1.7]])
        x_range = numpy.linspace(-1, 1.3, 3)
        y_range = numpy.linspace(-1.5, 0.55, 3)
        # targets = numpy.array([[-1.2, 0.75], [0, 0.75], [1.5, 0.75], [1.5, 0], [1.5, -1.7], [0, -1.7], [-1.2, -1.7], [-1.2, 0]])
        self.alternative_targets = Utilities.make_grid_list(x_range, y_range)

        self.tracker = tracker

        ## Bind robot proxies
        self.logger.write('Binding motion proxy')
        self.motion = ALProxy("ALMotion", address, 9559)
        self.motion.setStiffnesses("Body", 1.0)
        self.logger.write('Binding voice proxy')
        self.voice = ALProxy("ALTextToSpeech", address, 9559)
        self.motion_configuration = {}
        self.animated_speech = False
        self.logger.write('Binding posture proxy')
        self.posture = ALProxy("ALRobotPosture", address, 9559)
        self.logger.write('Binding battery proxy')
        self.battery = ALProxy("ALBattery", address, 9559)
        self.logger.write('Binding memory proxy')
        self.memory = ALProxy("ALMemory", address, 9559)
        self.logger.write('Binding system proxy')
        self.system = ALProxy("ALSystem", address, 9559)
        self.logger.write('Binding tracker proxy')
        self.landmarks = ALProxy("ALTracker", address, 9559)
        self.landmarks.unregisterAllTargets()
        self.logger.write('Binding audio player proxy')
        self.audio = ALProxy("ALAudioDevice", address, 9559)
        self.audio.setOutputVolume(75)
        self.logger.write('Binding speech recognition proxy')
        self.recognition = ALProxy("ALSpeechRecognition", address, 9559)
        self.stop_listen()
        self.logger.write('Switching off awareness')
        self.awareness = ALProxy("ALBasicAwareness", address, 9559)
        self.awareness.stopAwareness()
        self.behavior = ALProxy("ALBehaviorManager", address, 9559)

        ## get robot name
        self.nao_name = self.system.robotName()
        ## start safety monitor
        self.panic_button = False  # software panic button
        self.monitor_detection = [0, '']
        self.background_checks = [True, True, True]
        self.background_process = threading.Thread(target=self.background_monitor)
        self.exit_background_monitor = False
        self.background_process.start()

        ## set head position
        self.logger.write('Set head orientation')
        self.set_head_position()
        ## for motion thread
        self.exit_motion_flag = False
        self.motion_process = threading.Thread(target=self.execute_motion, args=(0, 0, False))
        ## for rotation thread
        self.exit_rotation_flag = False
        self.rotation_process = threading.Thread(target=self.execute_rotation, args=[0])
        ## start figure
        self.figure = pyplot.figure('Robot_' + self.robot_name)
        self.axis1 = self.figure.add_subplot(111)

        ## received command
        self.command = None

    def save_state(self, file_name):
        target = numpy.array([self.desired_x, self.desired_y])
        plot_path = numpy.vstack((self.position[0:2], target))
        data = {}
        data['speed_factor'] = self.speed_factor
        data['ip_address'] = self.ip_address
        data['robot_name'] = self.robot_name
        data['position'] = self.position
        data['velocity'] = self.velocity
        data['rotation'] = self.rotation
        data['current_angle'] = self.current_angle
        data['desired_x'] = self.desired_x
        data['desired_y'] = self.desired_y
        data['safety_distance'] = self.safety_distance
        data['target'] = target
        data['plot_path'] = plot_path
        io.savemat(file_name, data)

    def is_moving(self):
        return self.motion_process.is_alive()

    def update_background_checks(self, index, value):
        self.background_checks[index] = value
        self.panic_button = False
        self.monitor_detection = [0, '']
        self.logger.write('Background checks updated: ' + str(self.background_checks))

    def point_direction_world(self, world_x, world_y):
        result = self.world2robot(world_x, world_y)
        robot_x = result[0]
        robot_y = result[1]
        self.point_direction(robot_x, robot_y)

    def point_direction(self, robot_x, robot_y):
        vector_tuple = (robot_x, robot_y, 0)
        effector = 'RArm'
        if robot_y > 0: effector = 'LArm'
        self.landmarks.pointAt(effector, vector_tuple, 2, 0.5)

    def point_to_naomark(self, mark_id, start_azimuth=0, start_elevation=0, size=0.06):
        target_name = "LandMark"

        self.motion.waitUntilMoveIsFinished()
        self.landmarks.stopTracker()
        self.landmarks.unregisterAllTargets()

        self.landmarks.registerTarget(target_name, [size, [mark_id]])
        self.landmarks.setMode("Head")
        self.landmarks.setEffector("None")
        self.landmarks.track(target_name)
        while 1:
            self.set_head_position(start_azimuth, start_elevation, False)
            position = self.landmarks.getTargetPosition(2)
            if len(position) > 0: break
            time.sleep(0.25)
        self.landmarks.stopTracker()
        self.landmarks.unregisterAllTargets()
        self.set_head_position(start_azimuth, start_elevation, True)
        self.landmarks.pointAt('RArm', position, 2, 0.25)
        self.motion.waitUntilMoveIsFinished()
        self.logger.write('Marker ' + str(mark_id) + ' found at ' + str(position))
        return position

    def set_angle(self, angle_name, target_angle, blocking=True):
        target_angle = numpy.deg2rad(target_angle)
        self.motion.setAngles(angle_name, target_angle, 1)
        counter = 0
        while blocking:
            self.motion.setAngles(angle_name, target_angle, 1)
            time.sleep(0.05)
            current_angle = self.motion.getAngles(angle_name, True)
            delta = current_angle - target_angle
            delta = abs(delta)
            if delta < 0.087: break
            if counter > 50: break
            counter += 1
            # self.logger.write('Done setting head orientation')

    def set_head_position(self, yaw=0, pitch=0, blocking=True):
        # pitch -= 14  # offset for zero
        # self.logger.write('Setting head orientation')
        names = ['HeadYaw', 'HeadPitch']
        current_angles = self.motion.getAngles(names, True)
        if yaw is None: yaw = numpy.rad2deg(current_angles[0])
        if pitch is None: pitch = numpy.rad2deg(current_angles[1])
        target_angles = numpy.array([yaw, pitch])
        target_yaw = numpy.deg2rad(yaw)
        target_pitch = numpy.deg2rad(pitch)
        self.motion.setAngles(names, [target_yaw, target_pitch], 0.1)
        while blocking:
            self.motion.setAngles(names, [target_yaw, target_pitch], 0.1)
            time.sleep(0.05)
            current_angles = self.motion.getAngles(names, True)
            current_angles = numpy.array(current_angles)
            current_angles = numpy.rad2deg(current_angles)
            delta = current_angles - target_angles
            delta = numpy.abs(delta)
            delta = numpy.max(delta)
            if delta < 5: break
            # self.logger.write('Done setting head orientation')

    def world2robot(self, world_x, world_y, localize=True):
        if localize: self.localize_self()
        robot_position = self.position
        robot_position[2] = 0
        angle = self.current_angle
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

    def robot2world(self, robot_x, robot_y, localize=True):
        if localize: self.localize_self()
        robot_position = self.position
        robot_position[2] = 0
        angle = self.current_angle
        robot_cds = numpy.array([robot_x, robot_y, 0])
        world = Rotate.rotate_z(angle, robot_cds)
        world = world[0]
        world = world + robot_position
        world = numpy.asarray(world)
        world = world.flatten()
        return world

    def get_direction(self, world_x, world_y):
        theta = numpy.arctan2(world_y, world_x)
        theta = numpy.rad2deg(theta)
        return theta

    def get_axes(self, length=0.5):
        x_axis = self.robot2world(length, 0)
        y_axis = self.robot2world(0, length)
        robot_x_axis = numpy.array([[self.position[0], self.position[1]], [x_axis[0], x_axis[1]]])
        robot_y_axis = numpy.array([[self.position[0], self.position[1]], [y_axis[0], y_axis[1]]])
        return robot_x_axis, robot_y_axis

    def test_coordinate_conversion(self):
        self.localize_self()
        time.sleep(2)
        x_axis, y_axis = self.get_axes()
        dx = numpy.array([-1, 0, 1])
        dy = numpy.array([-1, 0, 1])
        dx, dy = numpy.meshgrid(dx, dy)
        dx = dx.flatten()
        dy = dy.flatten()
        nr = dx.size

        pyplot.close('all')
        pyplot.hold(True)
        pyplot.plot(x_axis[:, 0], x_axis[:, 1], 'r')
        pyplot.plot(y_axis[:, 0], y_axis[:, 1], 'g')

        for index in range(0, nr):
            world_x = dx[index]
            world_y = dy[index]
            converted = self.world2robot(world_x, world_y)
            converted_back = self.robot2world(converted[0], converted[1])
            text0 = Utilities.num2str(converted[0], 1)
            text1 = Utilities.num2str(converted[1], 1)
            text2 = Utilities.num2str(converted[2], 0)
            text = text0 + ', ' + text1 + ', ' + text2
            pyplot.plot(world_x, world_y, 'ko')
            pyplot.text(world_x + 0.1, world_y + 0.1, text)
            print world_x,
            print world_y,
            print '\t-->',
            print Utilities.num2str(converted[0], 2),
            print Utilities.num2str(converted[1], 2),
            print '\t\t-->',
            print Utilities.num2str(converted_back[0], 2),
            print Utilities.num2str(converted_back[1], 2)

        pyplot.hold(False)
        pyplot.axis('equal')
        pyplot.xlim((-1.5, 1.5))
        pyplot.ylim((-1.5, 1.5))
        pyplot.pause(0.5)
        pyplot.show()
        pyplot.savefig('coordinate_test.png')

    def clean_up(self):
        self.logger.write('Starting clean up process:')
        self.stop_robot()
        self.stop_listen()
        self.exit_background_monitor = True

    def get_touch_data(self):
        data = self.memory.getTimestamp('TouchChanged')
        surfaces = data[0]
        states = []
        names = []
        for surface in surfaces:
            states.append(surface[1])
            names.append(surface[0])
        timestamp = data[1] + data[2] * math.pow(10.0, -6)
        state = max(states)
        touch = {'timestamp': timestamp, 'surfaces': names, 'state': state, 'surface_states': states}
        return touch

    def stop_listen(self):
        try:
            self.recognition.unsubscribe("ASR")
            self.logger.write('Unsubscribed from speech recognition')
        except:
            self.logger.write('No previous subscription to speech recognition')

    def start_listen(self, words):
        self.logger.write('Start Listening for words: ' + str(words))
        self.stop_listen()
        self.recognition.setLanguage("English")
        self.recognition.setVocabulary(words, False)
        self.recognition.subscribe("ASR")

    def get_recognized_words(self):
        data = self.memory.getTimestamp('WordRecognized')
        data = data[0]
        word = data[0]
        confidence = data[1]
        return word, confidence

    def detect_feet_bumpers(self):
        data = self.get_touch_data()
        surfaces = data['surfaces']
        state = data['state']
        touched = False
        if 'LFoot/Bumper/Left' in surfaces: touched = True
        if 'LFoot/Bumper/Right' in surfaces: touched = True
        if 'RFoot/Bumper/Left' in surfaces: touched = True
        if 'RFoot/Bumper/Right' in surfaces: touched = True
        if not state: touched = False
        return touched

    def status_message(self, print_message=False):
        battery = self.get_battery_level()
        robot_name = self.robot_name
        nao_name = self.nao_name
        ip = self.ip_address
        panic = self.panic_button
        message = str(battery) + ' ' + nao_name + ' ' + robot_name + ' ' + ip + ' ' + str(panic)
        if print_message: self.logger.write(message)
        return message

    def background_monitor(self):
        rate = 1.0
        interval = 1.0 / float(rate)
        while True:
            start = time.time()
            if self.exit_background_monitor: break
            if self.panic_button: self.stop_robot()
            if not self.panic_button:
                ## Touch data
                touch_state = self.detect_feet_bumpers()
                if touch_state and self.background_checks[0]:
                    self.logger.write('SAFETY ISSUE: touch detected')
                    self.panic_button = True
                    self.monitor_detection = [1, 'touch']
                ## Battery level
                data = self.get_battery_level()
                if data < 10 and self.background_checks[1]:
                    self.logger.write('SAFETY ISSUE: low battery')
                    self.panic_button = True
                    self.monitor_detection = [2, 'battery']
                ## Too close to object
                distance = 1000
                if self.tracker is not None: distance, position, name = self.get_closest_obstacle(arena_perimeter=False)
                if distance < self.safety_distance and self.background_checks[2]:
                    self.logger.write('SAFETY ISSUE: too close to ' + name + ', ' + Utilities.num2str(distance, 2))
                    self.panic_button = True
                    self.monitor_detection = [3, name]
            end = time.time()
            rest = interval - (end - start)
            if rest > 0: time.sleep(rest)
        self.logger.write('Background monitor stopped')

    def stop_robot(self):
        if self.is_moving():
            self.logger.write('Stopping robot movement')
            self.stop_rotation_process()
            self.stop_motion_process()
            self.motion.moveToward(0, 0, 0)  # Stops Move task at next double support

    def get_battery_level(self, print_message=False):
        charge = self.battery.getBatteryCharge()
        if print_message: self.logger.write('Robot charge: ' + str(charge))
        return charge

    def switch_animation(self, on):
        if not on:
            self.logger.write('Switch off animated speech')
            self.voice = ALProxy("ALTextToSpeech", self.ip_address, 9559)
            self.animated_speech = False
        if on:
            self.logger.write('Switch on animated speech')
            self.voice = ALProxy("ALAnimatedSpeech", self.ip_address, 9559)
            self.motion_configuration = {"bodyLanguageMode": "contextual"}
            self.animated_speech = True

    def set_pose(self, posture_label, blocking=False, speed=0.5):
        if blocking:
            self.posture.goToPosture(posture_label, speed)
            self.motion.waitUntilMoveIsFinished()
        if not blocking:
            self.posture.post.goToPosture(posture_label, speed)

    def speak_text(self, text, blocking=False):
        if not self.animated_speech:
            if blocking:
                self.voice.say(text)
            if not blocking:
                self.voice.post.say(text)
        if self.animated_speech:
            if blocking:
                self.voice.say(text, self.motion_configuration)
            if not blocking:
                self.voice.post.say(text, self.motion_configuration)

    def localize_self(self, print_message=False, head_yaw=None):
        if self.tracker is not None:
            self.set_head_position(head_yaw, 0)
            yaw = self.motion.getAngles(['HeadYaw'], True)
            tracker_data = self.tracker.enquire(self.robot_name)
            self.velocity = tracker_data['velocity']
            self.position = tracker_data['position']
            self.rotation = tracker_data['rotation']

            self.current_x = self.position[0]
            self.current_y = self.position[1]
            self.current_angle = self.rotation[2] - yaw
            self.current_angle = Utilities.wrap180(self.current_angle)[0]

            x_position = Utilities.num2str(self.current_x)
            y_position = Utilities.num2str(self.current_y)
            angle = Utilities.num2str(self.current_angle)
            if print_message:
                message = 'I am at [' + x_position + ',' + y_position + '] with angle ' + angle
                self.logger.write(message)
        if self.tracker is None:
            self.set_head_position(head_yaw, 0)
            self.velocity = 0
            self.position = [0, 0, 0]
            self.current_x = 0
            self.current_y = 0
            self.current_angle = 0
            message = 'WARNING: dummy localization'
            self.logger.write(message)

    def localize_obstacles(self, arena_perimeter=True):
        names = self.tracker.all_objects[:]
        names.remove(self.robot_name)
        if names == []: return False
        positions = numpy.array([0, 0, 0])
        velocities = numpy.array([0, 0, 0])
        for obstacle in names:
            position = self.tracker.get_position(obstacle)
            velocity = self.tracker.get_velocity(obstacle)
            positions = numpy.vstack((positions, position))
            velocities = numpy.vstack((velocities, velocity))
        if arena_perimeter:
            arena = Utilities.get_perimeter(self.arena_points)
            n = len(arena[:, 0])
            names += ['Arena'] * n
            positions = numpy.vstack((positions, arena))
            velocities = numpy.vstack((velocities, arena * 0))
        positions = positions[1:, :]
        velocities = velocities[1:, :]
        # predict n seconds into the future
        occupied_positions = numpy.copy(positions)
        for time_step in [2, 4, 6]:
            predicted_positions = positions + (time_step * velocities)
            occupied_positions = numpy.vstack((occupied_positions, predicted_positions))

        data = {}
        data['positions'] = positions
        data['velocities'] = velocities
        data['occupied'] = occupied_positions
        data['names'] = names
        return data

    def get_closest_obstacle(self, arena_perimeter=True):
        if self.position is None: return 9999, None, None
        data = self.localize_obstacles(arena_perimeter)
        if not data: return 9999, None, None
        obstacles = data['positions']
        obstacle_names = data['names']
        distances = Utilities.distance2points(self.position, obstacles)
        min_index = numpy.argmin(distances)
        min_distance = distances[min_index]
        min_name = obstacle_names[min_index]
        position = obstacles[min_index, :]
        return min_distance, position, min_name

    def go_to_object(self, goal, safety_margin='10%', blocking=False, obstacle_avoidance=False, delay=0):
        self.localize_self()
        safety_margin = str(safety_margin)
        goal_position = self.tracker.get_position(goal)

        if safety_margin.endswith('%'):
            safety_margin = safety_margin.rstrip('%')
            safety_percentage = float(safety_margin)
            vector_to_goal = (goal_position - self.position) * (float(safety_percentage) / 100.0)
            position_to_go = goal_position - vector_to_goal
            desired_x = position_to_go[0]
            desired_y = position_to_go[1]

        elif safety_margin.endswith('m'):
            safety_margin = safety_margin.rstrip('m')
            safety_margin = float(safety_margin)
            vector_to_robot = (self.position - goal_position)
            vector_to_robot_norm = Utilities.normalize(vector_to_robot)
            vector_to_robot_norm = vector_to_robot_norm * safety_margin
            position_to_go = goal_position + vector_to_robot_norm
            desired_x = position_to_go[0]
            desired_y = position_to_go[1]

        else:
            self.logger.write('WARNING: could not interpret safety margin specification')
            return
        self.logger.write('Go to object: ' + goal)
        self.go_to_position(desired_x, desired_y, blocking, obstacle_avoidance, delay)

    def go_to_orientation(self, desired_angle, blocking=False):
        self.stop_motion_process()
        self.stop_rotation_process()
        self.logger.write('Go to orientation: ' + str(desired_angle))
        self.exit_rotation_flag = False
        self.rotation_process = threading.Thread(target=self.execute_rotation, args=[desired_angle])
        self.rotation_process.start()
        if blocking: self.rotation_process.join()

    def stop_rotation_process(self):
        while self.rotation_process.is_alive():
            self.logger.write('Stop rotation process')
            self.exit_rotation_flag = True
            self.rotation_process.join(0.1)

    def go_to_position(self, desired_x, desired_y, blocking=False, avoid_obstacles=False, delay=0):
        self.stop_rotation_process()
        self.stop_motion_process()
        x_string = Utilities.num2str(desired_x)
        y_string = Utilities.num2str(desired_y)
        target = x_string + ', ' + y_string
        self.logger.write('Go to position: ' + target)
        self.exit_motion_flag = False
        self.motion_process = threading.Thread(target=self.execute_motion, args=(desired_x, desired_y, avoid_obstacles, None, delay))
        self.motion_process.start()
        if blocking: self.motion_process.join()

    def stop_motion_process(self):
        counter = 0
        while self.motion_process.is_alive():
            if counter == 0: self.logger.write('Stop motion process')
            self.exit_motion_flag = True
            self.motion_process.join(0.1)
            counter+=1

    def movement_correction(self, localize=False, do_plot=True):
        if localize:self.localize_self()
        target = self.get_desired_target()[0:2]
        obstacle_data = self.localize_obstacles(arena_perimeter=False)
        obs_pos = obstacle_data['positions']
        occupied = obstacle_data['occupied']

        # remove obstacles that are the destination
        distances = Utilities.distance2points(target, obs_pos[:, 0:2])
        far_enough = distances > self.safety_distance
        sel_obs_pos = obs_pos[far_enough, :]

        if do_plot:
            self.axis1.clear()
            self.axis1.hold(True)
            self.axis1.scatter(obs_pos[:, 0], obs_pos[:, 1], marker='s', s=50, c='red', alpha=0.25)
            self.axis1.scatter(sel_obs_pos[:, 0], sel_obs_pos[:, 1], marker='s', s=50, c='red', alpha=1)
            self.axis1.scatter(occupied[:, 0], occupied[:, 1], marker='s', s=30, c='red', alpha=0.25)
            self.axis1.scatter(self.current_x, self.current_y, marker='o', s=50, c='black')
            self.axis1.scatter(target[0], target[1], marker='o', s=50, c='red')

        # find the clearance and deviances for each alternative path
        alternative_targets = numpy.copy(self.alternative_targets)
        alternative_targets = numpy.vstack((target, alternative_targets))
        nr_positions = alternative_targets.shape[0]
        deviances = numpy.zeros(nr_positions)
        clearances = numpy.zeros(nr_positions)
        for index in range(0, nr_positions):
            potential_target = alternative_targets[index, :]
            distance_potential_target = numpy.linalg.norm(self.position[0:2] - potential_target)
            path = Utilities.interpolate_path(self.position[0:2], potential_target, 0, distance_potential_target, 25)
            # if do_plot: self.axis1.scatter(potential_target[0], potential_target[1], marker='d', alpha=0.5, s=50)
            distance_matrix = spatial.distance.cdist(path, sel_obs_pos[:, 0:2])
            distance_matrix = distance_matrix > self.safety_distance
            deviance = numpy.linalg.norm(target[0:2] - potential_target)
            clearance = numpy.mean(distance_matrix)
            clearances[index] = clearance
            deviances[index] = deviance
            if do_plot:
                self.axis1.plot(path[:, 0], path[:, 1], c='black', alpha=0.25, linewidth=1)
                self.axis1.text(path[-1, 0], path[-1, 1], Utilities.num2str(clearance, 2))

        max_clearance = numpy.max(clearances)
        deviances[clearances < max_clearance] = 100000
        best_index = numpy.argmin(deviances)
        alternative_target = alternative_targets[best_index, :]
        alternative_target = numpy.hstack((alternative_target, 0))

        if do_plot:
            self.axis1.scatter(alternative_target[0], alternative_target[1], marker='o', s=150, c='green', alpha=0.25)
            self.axis1.hold(False)
            self.axis1.set_xlim((-2, 2))
            self.axis1.set_ylim((-2, 2))

        return alternative_target

    def scale_motion_config(self, scale=None):
        if scale is None: scale = self.speed_factor
        if scale > 1.0: scale = 1.0
        if scale < 0.0: scale = 0.0
        settings_default = self.motion.getMoveConfig('Default')
        settings_min = self.motion.getMoveConfig('Min')
        settings_max = self.motion.getMoveConfig('Max')
        new_config = []
        to_scale = ['MaxStepX', 'MaxStepY']
        for index in range(0, len(settings_default)):
            nm = settings_default[index][0]
            mn = settings_min[index][1]
            mx = settings_max[index][1]
            df = settings_default[index][1]
            scaled = df
            if nm in to_scale: scaled = df * scale
            if scaled < mn: scaled = mn
            if scaled > mx: scaled = mx
            new_config.append([nm, scaled])
        return new_config

    def get_desired_target(self, coordinate_index=None):
        target = numpy.array([self.desired_x, self.desired_y, 0])
        if coordinate_index is not None: return target[coordinate_index]
        return target

    def execute_rotation(self, desired_angle, frequency=0.5):
        self.motion.stopMove()
        self.motion.waitUntilMoveIsFinished()
        self.logger.write('Starting Rotation')
        period = 1.0 / frequency
        self.set_head_position()
        while True:
            start = time.time()
            self.localize_self()
            angle = self.current_angle
            delta = desired_angle - angle
            delta = Utilities.wrap180(delta)
            if abs(delta) < 5: break
            if self.exit_rotation_flag: break
            v_r = sign(delta)
            speed = numpy.interp(delta, [-1000, -180, -10, 10, 180, 1000], [0.25, 0.25, 0.10, 0.10, 0.25, 0.25])[0]
            self.motion.moveInit()
            self.motion.moveToward(0, 0, v_r * speed)
            time_left = period - (time.time() - start)
            self.monitor_gyroscope(time_left)
        self.motion.moveToward(0, 0, 0)

    def execute_motion(self, desired_x, desired_y, obstacle_avoidance=False, frequency=1, delay=0):
        # setting default when calling the function using threading (keywords can not be used then)
        if obstacle_avoidance is None: obstacle_avoidance = False
        if frequency is None: frequency = 1
        if delay is None: delay = 0

        time.sleep(delay)

        period = 1.0/frequency
        head_azimuth = 0
        self.desired_x = desired_x
        self.desired_y = desired_y
        self.logger.write('Entering motion process')
        self.logger.write('Threshold: ' + str(self.arrival_threshold))
        while True:

            start = time.time()
            self.localize_self(print_message=False, head_yaw=head_azimuth)

            # Get Obstacle Avoidance Target
            self.axis1.clear()
            target = numpy.array([self.desired_x, self.desired_y, 0])
            self.original_target = numpy.copy(target)
            if obstacle_avoidance: target = self.movement_correction()
            self.corrected_target = numpy.copy(target)
            goal_distance = numpy.linalg.norm(self.position[0:2] - self.original_target[0:2])

            # Exit conditions
            if self.exit_motion_flag:
                self.logger.write('exit_motion_flag = True: Exiting motion process')
                break
            if goal_distance < self.arrival_threshold:
                self.logger.write('Arrived!')
                self.motion.moveToward(0, 0, 0)
                break
            if self.panic_button:
                self.logger.write('Panic button: Exiting motion process')
                self.motion.moveToward(0, 0, 0)
                break

            # make movement vector
            relative_target_cds = self.world2robot(target[0], target[1])
            movement_vector = relative_target_cds[0:2]
            #print movement_vector,
            max_value = numpy.max(numpy.abs(movement_vector))
            movement_vector /= max_value
            #print movement_vector

            movement_rotation = relative_target_cds[2]
            if goal_distance < 0.25: movement_rotation = 0

            # get new head azimuth
            head_azimuth = movement_rotation
            if head_azimuth < -45: head_azimuth = -45
            if head_azimuth > 45: head_azimuth = 45

            # make sure we take small steps
            speed = self.speed_factor
            if abs(movement_rotation) < 10: movement_rotation = 0
            v_x = movement_vector[0] * speed
            v_y = movement_vector[1] * speed
            v_r = sign(movement_rotation) * speed * 0.25
            self.motion.moveInit()
            self.motion.moveToward(v_x, v_y, v_r)

            # pause and check for swaying
            time_left = period - (time.time() - start)
            self.monitor_gyroscope(time_left)
        self.logger.write('Exited motion process thread')

    def monitor_gyroscope(self, max_sample_period):
        if max_sample_period < 0: return 0
        max_gyro = 0
        start = time.time()
        while (time.time() - start) < max_sample_period:
            data = self.memory.getData('Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value')
            data = abs(data)
            max_gyro = max([max_gyro, data])
            if max_gyro > self.sway_threshold: break
        if max_gyro > self.sway_threshold:
                self.logger.write('Swaying detected: Gyro value ' + num2str(max_gyro))
                self.motion.moveToward(0, 0, 0)
                self.motion.waitUntilMoveIsFinished()
                self.set_pose('StandInit', blocking=True, speed=0.5)
        return max_gyro




