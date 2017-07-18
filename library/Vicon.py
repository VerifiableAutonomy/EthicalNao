#!/usr/bin/python

# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3.0 of the License, or (at your option) any later version.
#
# The library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# (c) Shai Revzen, U Penn, 2010

import struct
import socket
import time
import threading

import numpy

import Utilities


#DEFAULT_HOST = "164.11.73.31"
DEFAULT_HOST = "192.168.2.2"
DEFAULT_PORT = 800
DEFAULT_FPS = 50


def data2struct(names, data):
    length = len(names)
    dct = {}
    for x in range(0, length):
        name = names[x]
        name = name.replace(' ', ':')
        name = name.replace('>', '')
        name = name.replace('<', '')
        dta = data[x]
        dct[name] = dta
    return dct


class Streamer():
    def __init__(self):
        reader = ViconReader()
        self.names = reader.connect()
        self.streamer = reader.stream()
        self.delta = 0
        self.data = None
        self.update()

    def __del__(self):
        self.streamer.close()

    def update(self):
        start = time.time()
        data = self.streamer.next()
        stop = time.time()
        data = data2struct(self.names, data)
        self.delta = stop - start
        self.data = data

    def get_objects(self):
        names = self.names
        objects = []
        for x in names:
            x = x.split(':')
            objects.append(x[0])
        objects = list(set(objects))
        return objects

    def get_object_orientation(self, current_object, set_z_zero=True):
        labels = ['T-X', 'T-Y', 'T-Z', 'A-X', 'A-Y', 'A-Z']
        translation = []
        rotation = []
        current_object = current_object + ':' + current_object + ':'
        for lbl in labels:
            name = current_object + lbl
            value = self.data[name]
            if lbl in ['T-X', 'T-Y', 'T-Z']:
                if lbl == 'T-Z' and set_z_zero: value = 0.0
                value /= 1000.0
                translation.append(value)
            if lbl in ['A-X', 'A-Y', 'A-Z']:
                value = numpy.rad2deg(value)
                rotation.append(value)
        translation = numpy.array(translation)
        rotation = numpy.array(rotation)
        orientation = {'translation': translation, 'rotation': rotation}
        return orientation


class Tracker:
    def __init__(self, tracked_objects, virtual_objects, tracker_name):
        self.streamer = Streamer()
        self.tracked_objects = tracked_objects
        self.virtual_objects = virtual_objects
        self.buffer_size = 50
        self.sample_rate = 25
        self.data = {}
        self.thread = None
        self.halt = False
        self.counter = 0
        self.filled = False
        self.name = tracker_name
        self.logger = Utilities.Logger(tracker_name)
        for name in self.tracked_objects: self.data[name] = numpy.empty((self.buffer_size, 7))
        self.all_objects = tracked_objects[:] + virtual_objects.keys()

        self.start()

    def update_buffer(self):
        self.streamer.update()
        self.counter += 1
        if self.counter > self.buffer_size: self.filled = True
        # if self.counter % 100 == 0: self.logger.write('Tracker running')
        for name in self.tracked_objects:
            timestamp = numpy.array(time.time())
            data = self.streamer.get_object_orientation(name)
            # print name, data
            translation = data['translation']
            rotation = data['rotation']
            line = numpy.hstack((timestamp, translation, rotation))
            current_data = self.data[name]
            new_data = numpy.vstack((current_data, line))
            new_data = new_data[1:, :]
            self.data[name] = new_data

    def track(self):
        delay = 1.0 / float(self.sample_rate)
        while 1:
            self.update_buffer()
            time.sleep(delay)
            if self.halt: break
        self.logger.write('Tracker stopped')

    def start(self):
        self.halt = False
        self.thread = threading.Thread(target=self.track)
        self.thread.start()
        self.logger.write('Tracker Started')
        self.logger.write('Waiting for tracker to be filled with data')
        while not self.filled: time.sleep(0.1)
        self.logger.write('Filled with data')

    def stop(self):
        self.halt = True

    def get_velocity(self, object_name):
        keys = self.virtual_objects.keys()
        if object_name in keys: return numpy.array([0, 0, 0])
        data = self.data[object_name]
        time_stamps = data[:, 0]
        time_stamps = time_stamps - numpy.min(time_stamps)
        positions_x = data[:, 1]
        positions_y = data[:, 2]
        positions_z = data[:, 3]
        intercept, slope_x = Utilities.my_regression(time_stamps, positions_x)
        intercept, slope_y = Utilities.my_regression(time_stamps, positions_y)
        intercept, slope_z = Utilities.my_regression(time_stamps, positions_z)
        velocity = numpy.array([slope_x, slope_y, slope_z])
        return velocity

    def get_speed(self, object_name):
        velocity = self.get_velocity(object_name)
        speed = numpy.linalg.norm(velocity[0:2])
        return speed

    def get_position(self, object_name):
        keys = self.virtual_objects.keys()
        if object_name in keys: return numpy.array(self.virtual_objects[object_name])
        data = self.data[object_name]
        shape = data.shape
        index = shape[0] - 1
        position = data[index, 1:4]
        return position

    def get_rotation(self, object_name):
        keys = self.virtual_objects.keys()
        if object_name in keys: return numpy.array([0, 0, 0])
        data = self.data[object_name]
        shape = data.shape
        index = shape[0] - 1
        rotation = data[index, 4:]
        return rotation

    def enquire(self, object_name):
        if self.counter < self.buffer_size + 1: raise StandardError, "The tracker data is not filled"
        if self.halt: raise StandardError, "The tracker (" + self.name + ") is not running"
        velocity = self.get_velocity(object_name)
        position = self.get_position(object_name)
        rotation = self.get_rotation(object_name)
        speed = self.get_speed(object_name)
        data = {'velocity': velocity, 'position': position, 'rotation': rotation, 'speed': speed}
        return data


        # NOT MY CODE FROM HERE ON


class ViconReader(object):
    QUERY = (1, 0)
    INFO = (1, 1)
    START = (3, 0)
    STOP = (4, 0)
    DATA = (2, 1)

    def __init__(self):
        self.sock = None
        self.push = ''

    def _get(self, fmt):
        "Read data from socket based on format string, and parse it accordingly"
        N = struct.calcsize(fmt)
        # Start reading from push-back buffer
        buf = self.push[:min(len(self.push), N)]
        self.push = self.push[:len(buf)]
        while len(buf) < N:
            buf += self.sock.recv(N - len(buf))
        return struct.unpack(fmt, buf)

    def _parseInfo(self):
        "Parse an INFO packet, starting with byte after the header"
        N = self._get("=1L")[0]
        lst = []
        for _ in xrange(N):
            L = self._get("=1L")[0]
            lst.append(self._get("%ds" % L)[0])
        return lst

    def _parseData(self):
        "Parse a DATA packet, starting with byte after the header"
        N = self._get("=1L")[0]
        # print self._get("%dd" % (N*8))
        return self._get("%dd" % (N * 1))

    def _parse(self):
        "Parse an incoming packet"
        hdr = self._get("=2L")
        if hdr == self.__class__.DATA:
            return (hdr, self._parseData())
        elif hdr == self.__class__.INFO:
            return (hdr, self._parseInfo())
        # Failed -- need to resync
        self.push = struct.pack("=2L", *hdr)
        self._resync()

    def _resync(self):
        raise ValueError, "Lost synchronization on socket"

    def _cmd(self, hdr):
        "Command encoded as a 2-tuple header"
        self.sock.send(struct.pack("=2L", *hdr))

    def connect(self, host=DEFAULT_HOST, port=DEFAULT_PORT):
        # Connect the socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        # Send a query
        self._cmd(self.__class__.QUERY)
        # Loop until a query response is received
        while True:
            hdr, names = self._parse()
            if hdr == self.__class__.INFO:
                return names

    def stream(self, fps=DEFAULT_FPS):
        """
        Generator producing a stream of data packet payloads

        INPUTS
          fps -- expected fps, for throwing away stale data.
            If not set, all data will be returned. Set to the expected vicon
            frames-per-second data capture rate; typically 120
        """
        self._cmd(self.__class__.START)
        thresh = 0.5 / fps
        while True:
            # Make sure we enter data reading loop at least once
            last = time.time() + 1e6
            while time.time() < thresh + last:
                last = time.time()
                hdr, data = self._parse()
                # print str(data)
            if hdr == self.__class__.DATA:
                yield data

    def stop(self):
        "Tell Vicon to stop streaming"
        self._cmd(self.__class__.STOP)

    def close(self):
        "Close connection to Vicon"
        self.sock.close()
        self.sock = None

# import time
# S = Streamer()
# for i in range(0,1000):
# x = S.data['HUMAN:HUMAN:T-X']
# y = S.data['HUMAN:HUMAN:T-Y']
# print i,x,y
# time.sleep(1)
# S.update()
