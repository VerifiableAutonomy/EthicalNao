__author__ = 'dieter'

import time
import sys
import os
import os.path
import shutil
import math
import glob
import inspect

from PIL import ImageOps
from PIL import Image
from PIL import ImageFilter
import scipy.interpolate
import numpy
from matplotlib import pyplot

import Rotate


def print_dict(dictionary, current_depth=0, line_break = '<br>'):
    txt = ''
    for key, value in dictionary.iteritems():
        if isinstance(value, dict):
            txt += (current_depth + 1) * '*' + str(key) + line_break
            depth = current_depth + 1
            txt += print_dict(value, depth)
        else:
            txt += (current_depth + 1) * '>' + str(key) + '=' + str(value) + line_break
    return txt


def get_current_script():
    script_file = inspect.getfile(inspect.currentframe())
    return script_file


def is_between(point, point1, point2, delta=0.5):
    f = interpolate_path(point1, point2, 0, 1)
    xi = arange(0, 1, 0.05)
    line = f(xi)
    distances = distance2points(point, line)
    min_distance = numpy.min(distances)
    if min_distance < delta: return True
    return False


#def sigmoid(x, threshold=0.25, slope=10):
#    value = 1 / (1 + numpy.exp(- slope * (x - threshold)))
#    return value

def sigmoid(x, threshold=1, slope=5):
    value = 1 / (1 + numpy.exp(- slope * (x - threshold)))
    return value


def is_array(variable):
    return isinstance(variable, numpy.ndarray)


def set_numpy_style():
    numpy.set_printoptions(precision=3, suppress=True, linewidth=200)


def wrap360(angles):
    array_input = is_array(angles)
    if not array_input: angles = numpy.array([angles])
    positive_input = angles > 0
    angles = numpy.mod(angles, 360)
    zeros = angles == 0
    selected = numpy.minimum(positive_input, zeros)
    angles[selected] = 360
    return angles


def wrap180(angles):
    array_input = is_array(angles)
    if not array_input: angles = numpy.array([angles])
    c1 = angles < -180
    c2 = angles > 180
    c = numpy.maximum(c1, c2)
    angles[c] = wrap360(angles[c] + 180) - 180
    return angles


def make_experiment_tree(session_path):
    empty_folder(session_path)
    empty_folder(session_path + '/plots')
    empty_folder(session_path + '/images')
    empty_folder(session_path + '/matfiles')
    empty_folder(session_path + '/code')
    empty_folder(session_path + '/logs')


def make_grid_list(x_range, y_range):
    x_range, y_range = numpy.meshgrid(x_range, y_range)
    x_range = x_range.flatten()
    y_range = y_range.flatten()
    positions = numpy.vstack((x_range, y_range))
    positions = positions.transpose()
    return positions


def rotate_around_point(vector, point, alpha):
    vector = numpy.hstack((vector, 0))
    point = numpy.hstack((point, 0))
    vector = vector - point
    new, matrix = Rotate.rotate_z(alpha, vector)
    new = new + point
    new = numpy.asarray(new)
    new = new.flatten()
    return new[0:2]


def unique_rows(data):
    data = numpy.ascontiguousarray(data)
    data = data.round(decimals=3)
    n_cols = data.shape[1]
    dtype = data.dtype.descr * n_cols
    struct = data.view(dtype)
    uniq = numpy.unique(struct)
    uniq = uniq.view(data.dtype).reshape(-1, n_cols)
    return uniq


def get_perimeter(points, n=10):
    start = points[0, :]
    points = numpy.vstack((points, start))
    x_points = points[:, 0]
    y_points = points[:, 1]
    indices = range(0, len(x_points))
    indices_interpolate = numpy.linspace(0, max(indices), len(indices) * n)
    f_x = scipy.interpolate.interp1d(indices, x_points)
    f_y = scipy.interpolate.interp1d(indices, y_points)
    f_xi = f_x(indices_interpolate)
    f_yi = f_y(indices_interpolate)
    perimeter = numpy.vstack((f_xi, f_yi, f_yi * 0))
    perimeter = perimeter.transpose()
    perimeter = unique_rows(perimeter)
    return perimeter


def add_border(image, color, size=100):
    result = ImageOps.crop(image, border=size)
    result = ImageOps.expand(result, border=size, fill=color)
    return result


def replace_color(image, color_in, color_out, threshold=100, do_plot=False):
    im = numpy.array(image, dtype=numpy.float)

    d0 = color_in[0] - im[:, :, 0]
    d1 = color_in[1] - im[:, :, 1]
    d2 = color_in[2] - im[:, :, 2]
    d = d0 ** 2 + d1 ** 2 + d2 ** 2
    d = numpy.sqrt(d)

    foreground = numpy.ones_like(im)
    im[d < threshold] = numpy.array(color_out)
    foreground[d < threshold] = numpy.array([0, 0, 0])
    im = numpy.array(numpy.round(im), dtype=numpy.uint8)
    if do_plot:
        pyplot.close('all')
        pyplot.subplot(221)
        pyplot.hist(d.flatten(), bins=100)
        pyplot.subplot(222)
        pyplot.imshow(im)
        pyplot.subplot(223)
        pyplot.imshow(foreground)

    return im

def remove_background(im_file, threshold=100, do_plot=False):
    im = Image.open(im_file)
    im = im.filter(ImageFilter.MedianFilter(3))
    im = numpy.array(im, dtype=numpy.float)

    median = numpy.median(im, axis=(0, 1))

    d0 = median[0] - im[:, :, 0]
    d1 = median[1] - im[:, :, 1]
    d2 = median[2] - im[:, :, 2]
    d = d0 ** 2 + d1 ** 2 + d2 ** 2
    d = numpy.sqrt(d)

    foreground = numpy.ones_like(im)
    im[d < threshold] = numpy.array([0, 0, 0])
    foreground[d < threshold] = numpy.array([0, 0, 0])
    im = numpy.array(numpy.round(im), dtype=numpy.uint8)
    if do_plot:
        pyplot.close('all')
        pyplot.subplot(221)
        pyplot.hist(d.flatten(), bins=100)
        pyplot.subplot(222)
        pyplot.imshow(im)
        pyplot.subplot(223)
        pyplot.imshow(foreground)

    return im, foreground, median


def composite_image(files, threshold=80, color=False):
    im_file = files[0]
    image, foreground, median = remove_background(im_file, threshold)
    shape = image.shape
    if not color: color = (int(median[0]), int(median[1]), int(median[2]))
    image_shape = (int(shape[1]), int(shape[0]))
    composite = Image.new("RGB", image_shape, color)
    composite = numpy.array(numpy.round(composite), dtype=numpy.uint8)
    for file_name in files:
        print file_name
        image, foreground, median = remove_background(file_name, threshold)
        composite[foreground > 0] = image[foreground > 0]
    composite = Image.fromarray(composite)
    return composite, median


def copy_python_files(target):
    files = glob.glob('*.py')
    for f in files:
        print f
        src = os.path.join(os.getcwd(), f)
        tgt = target + '/' + f
        shutil.copy(src, tgt)


def scale(x, minimum=0, maximum=1):
    x = x.astype(float)
    x = x + numpy.min(x.flatten())
    max_val = numpy.max(x.flatten())
    if max_val == 0: return x * 0
    x = x / max_val
    r = maximum - minimum
    x = x * r
    x = x + minimum
    return x


def arange(start, stop, step):
    var = stop - start
    nr_steps = float(var) / float(step)
    nr_steps = int(math.ceil(nr_steps))
    r = numpy.linspace(start, stop, nr_steps)
    return r


def closest(x, array):
    distance = numpy.abs(array - x)
    min_index = numpy.argmin(distance)
    found = array[min_index]
    return found, min_index


def empty_folder(folder):
    if os.path.exists(folder): shutil.rmtree(folder)
    os.makedirs(folder)


def list2txt(lst, sep=','):
    text = ''
    for x in lst: text += str(x) + sep
    text = text.rstrip(sep)
    return text

def interpolate_path(p1, p2, t1, t2, n=None):
    p1 = p1.flatten()
    p2 = p2.flatten()
    plus_inf = 9999999
    min_inf = -9999999
    if t1 > t2: x = numpy.hstack((plus_inf, t1, t2, min_inf))
    if t2 >= t1: x = numpy.hstack((min_inf, t1, t2, plus_inf))
    y = numpy.vstack((p1, p1, p2, p2))
    f = scipy.interpolate.interp1d(x, y, axis=0)
    f.range = numpy.array([t1, t2])
    if n is None: return f
    xi = numpy.linspace(t1, t2, n)
    interpolated = f(xi)
    return interpolated


def distance2points(ref_point, points):
    ref_point = ref_point.flatten()
    points = numpy.asmatrix(points)
    ref_point = numpy.asmatrix(ref_point)
    number = points.shape[0]
    ref_point = numpy.tile(ref_point, (number, 1))
    delta = points - ref_point
    distances = numpy.linalg.norm(delta, axis=1)
    distances = numpy.asarray(distances)
    distances = distances.flatten()
    return distances


def perpendicular_distance(p0, p1, p2):
    # gives the perpendicular distance from point 0 to the line defined by p1-p2
    x0 = p0[0]
    x1 = p1[0]
    x2 = p2[0]

    y0 = p0[1]
    y1 = p1[1]
    y2 = p2[1]

    a = numpy.abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    b = numpy.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
    result = a / b
    return result


def num2str(number, digits=3):
    if isinstance(number, basestring): return str(number)
    stg = '{0:.' + str(digits) + 'f}'
    return stg.format(number)


def my_regression(x, y):
    x = x.flatten()
    y = y.flatten()
    x = x - numpy.min(x)
    delta_x = numpy.diff(x)
    delta_y = numpy.diff(y)
    slope = delta_y / delta_x
    slope = numpy.mean(slope)
    mn_x = numpy.mean(x)
    mn_y = numpy.mean(y)
    intercept = mn_y - slope * mn_x
    return intercept, slope


class Logger(object):
    def __init__(self, name, filename="Default.txt"):
        self.terminal = sys.stdout
        self.name = name
        self.filename = filename

    def write(self, text):
        asc_time = time.asctime()
        screen_message = self.name.ljust(20) + '> ' + text + '\n'
        log_message = [asc_time, self.name, text]
        log_message = list2txt(log_message, ';') + '\n'
        self.terminal.write(screen_message)
        log = open(self.filename, "a")
        log.write(log_message)
        log.close()


def angle_vectors(a, b):
    noise_a = numpy.random.rand(a.shape[0])*0.001
    noise_b = numpy.random.rand(b.shape[0])*0.001
    a_norm = numpy.linalg.norm(a + noise_a)
    b_norm = numpy.linalg.norm(b + noise_b)
    a = a / a_norm
    b = b / b_norm
    p = numpy.dot(a, b)
    angle = numpy.arccos(p)
    angle = numpy.rad2deg(angle)
    return angle


def normalize(a):
    a_norm = numpy.linalg.norm(a)
    if a_norm == 0: return a * 0
    normalized = a / a_norm
    return normalized


def normalize_array(a):
    a_norm = numpy.linalg.norm(a, axis=1)
    a_norm_mat = numpy.tile(a_norm, (a.shape[1], 1))
    a_norm_mat = numpy.transpose(a_norm_mat)
    normalized = a / a_norm_mat
    is_zero = a_norm == 0
    normalized[is_zero, :] = 0
    return normalized


def remove_stable(path):
    try:
        os.remove(path)
    except:
        print 'Could not remove: ', path


def padd_int(x, n=3):
    x = str(int(x))
    return x.zfill(n)
