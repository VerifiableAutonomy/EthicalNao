# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 14:02:36 2016

@author: dieter
"""

# import cPickle as pickle
import matplotlib.pyplot as pyplot
from scipy.signal import savgol_filter
import numpy
import PathPlanning
import copy
from scipy.interpolate import interp1d
from scipy.interpolate import RectBivariateSpline


def attention_map(centers_of_attention):
    centers_of_attention = numpy.reshape(centers_of_attention, (-1, 3))

    x_range = [-3, 3]
    y_range = [-3, 3]
    sd = 0.5

    x_range = numpy.arange(x_range[0], x_range[1], 0.1)
    y_range = numpy.arange(y_range[0], y_range[1], 0.1)

    nr_centers = centers_of_attention.shape[0]
    x_mat, y_mat = numpy.meshgrid(x_range, y_range)
    distance = numpy.ones(x_mat.shape) * 10000
    for index in range(0, nr_centers):
        position_x = centers_of_attention[index, 0]
        position_y = centers_of_attention[index, 1]
        current_distance = (x_mat - position_x) ** 2 + (y_mat - position_y) ** 2
        distance = numpy.minimum(distance, current_distance)
    attention = numpy.exp(- distance / (2 * sd ** 2))
    if nr_centers == 0: attention = numpy.ones(attention.shape)
    function = RectBivariateSpline(x_range, y_range, numpy.transpose(attention))
    result = {}
    result['attention_map'] = attention
    result['distance_map'] = distance
    result['x_range'] = x_range
    result['y_range'] = y_range
    result['x_mat'] = x_mat
    result['y_mat'] = y_mat
    result['centers'] = centers_of_attention
    result['function'] = function
    return result


def select(result, error_threshold):
    errors = numpy.array(result['errors'])
    steps = numpy.array(result['steps'])
    first = numpy.where(errors < error_threshold)
    first = numpy.min(first)
    steps = steps[first]
    steps_key = 'data_step_' + str(steps)
    selected_demonstration = result[steps_key]
    planned_x = selected_demonstration['planned_x_i']
    planned_y = selected_demonstration['planned_y_i']
    selected_waypoints = numpy.transpose(numpy.vstack((planned_x, planned_y)))
    selected_waypoints = selected_waypoints[1:, ]
    result = {}
    result['selected_waypoints'] = selected_waypoints
    result['selected_demonstration'] = selected_demonstration
    result['steps'] = steps
    result['steps_key'] = steps_key
    return result


def evaluate(demonstrator_travelled_path, graph, attention_function=None, add_patches=False, plot=False):
    graph = copy.copy(graph)
    attention_function = copy.copy(attention_function)
    travelled_x = demonstrator_travelled_path[:, 0]
    travelled_y = demonstrator_travelled_path[:, 1]
    travelled_x_smoothed = savgol_filter(travelled_x, 5, 1)
    travelled_y_smoothed = savgol_filter(travelled_y, 5, 1)

    # interpolate travelled path
    time = numpy.linspace(0, 1, numpy.size(travelled_x_smoothed))
    time_error = numpy.linspace(0, 1, 100)
    travelled_x_i_function = interp1d(time, travelled_x_smoothed)
    travelled_y_i_function = interp1d(time, travelled_y_smoothed)
    t_x_i_error = travelled_x_i_function(time_error)
    t_y_i_error = travelled_y_i_function(time_error)

    if attention_function is not None: attention = attention_function(t_x_i_error, t_y_i_error, grid=False)
    if attention_function is None: attention = 1

    errors = []
    step_range = [2, 3, 5, 9, 17]
    all_data = {}
    figure_handles = []
    for steps in step_range:
        print 'EVALUATING STEP ' + str(steps)
        time_i = numpy.linspace(0, 1, steps)
        travelled_x_i = travelled_x_i_function(time_i)
        travelled_y_i = travelled_y_i_function(time_i)

        # plan path trough way points
        waypoints = numpy.transpose(numpy.vstack((travelled_x_i, travelled_y_i)))
        if add_patches: graph.set_patches(waypoints)
        planning_result = graph.find_path_via(waypoints, False)
        planned_smoothed = planning_result['smoothed']
        planned_x = planned_smoothed[:, 0]
        planned_y = planned_smoothed[:, 1]

        # interpolate planned path
        time = numpy.linspace(0, 1, numpy.size(planned_x))
        planned_x_i_function = interp1d(time, planned_x)
        planned_y_i_function = interp1d(time, planned_y)
        planned_x_i = planned_x_i_function(time_i)
        planned_y_i = planned_y_i_function(time_i)

        # calculate error
        p_x_i_error = planned_x_i_function(time_error)
        p_y_i_error = planned_y_i_function(time_error)
        error = ((p_x_i_error - t_x_i_error) ** 2 + (p_y_i_error - t_y_i_error) ** 2) * attention
        error = numpy.sqrt(numpy.sum(error))
        if not planning_result['success']: error = max(errors)
        errors.append(error)

        partial_data = {}
        partial_data['time_i'] = time_i
        partial_data['planned_x'] = planned_x
        partial_data['planned_y'] = planned_y
        partial_data['planned_x_i'] = planned_x_i
        partial_data['planned_y_i'] = planned_y_i
        partial_data['error_data'] = [p_x_i_error, t_x_i_error, p_y_i_error, t_y_i_error]
        all_data['data_step_' + str(steps)] = partial_data
        if plot:
            handle = graph.plot_network()
            handle = handle.number
            figure_handles.append(handle)
            pyplot.hold(True)
            pyplot.plot(travelled_x, travelled_y, color='red')
            pyplot.plot(travelled_x_smoothed, travelled_y_smoothed, color='red', linewidth=3, alpha=0.25)
            pyplot.plot(travelled_x_i, travelled_y_i, '.', color='red', markersize=10)
            pyplot.plot(planned_smoothed[:, 0], planned_smoothed[:, 1], color='green', linewidth=3, alpha=0.5)
            pyplot.plot(planned_x_i, planned_y_i, '.', color='green', markersize=10)
            pyplot.hold(False)
    all_data['figure_handles'] = figure_handles
    all_data['errors'] = numpy.array(errors)
    all_data['steps'] = numpy.array(step_range)
    all_data['attention'] = attention
    all_data['attention_function'] = attention_function
    all_data['travelled_x_i'] = travelled_x_i
    all_data['travelled_y_i'] = travelled_y_i
    all_data['travelled_x_i_error'] = t_x_i_error
    all_data['travelled_y_i_error'] = t_y_i_error
    all_data['travelled_x'] = travelled_x
    all_data['travelled_y'] = travelled_y
    all_data['travelled_x_s'] = travelled_x_smoothed
    all_data['travelled_y_s'] = travelled_y_smoothed
    return all_data


    # f = open('/home/dieter/Desktop/saved.p', 'rb')
    # saved_data = pickle.load(f)
    # f.close()

    # pyplot.close('all')
    # result = evalute(saved_data,True)



    # pyplot.show()
