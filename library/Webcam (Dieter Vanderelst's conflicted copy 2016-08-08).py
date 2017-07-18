__author__ = 'dieter'

import urllib2
import time
import os.path
import threading

from PIL import Image
from matplotlib import pyplot

# Get equations http://arohatgi.info/WebPlotDigitizer/app/?

class Webcam:
    def __init__(self):
        self.ip_address = '164.11.73.35'
        self.path = '/img/snapshot.cgi?size=3&quality=1'
        self.address = 'http://' + self.ip_address + self.path
        self.origin = (386.0, 193.0)
        self.scale = 180.0  # pixels per meter
        self.image = None
        self.path = 'snapshot.png'
        self.stop_capture_process = False
        self.capture_process = threading.Thread(target=self.capture, args=(None, None))

    def pixel2world(self, x_pixel, y_pixel):
        x_data = (0.005699339538127619) * x_pixel + (0.0000346464409612615) * y_pixel + (-1.991306073122779)
        y_data = (0.00007795449216284026) * x_pixel + (-0.005837925301972666) * y_pixel + (1.038031420789668)
        return x_data, y_data

    def world2pixel(self, x_data, y_data):
        x_pixel = (175.44468546637745) * x_data + (1.0412147505422809) * y_data + (348.28325403946064)
        y_pixel = (2.3427331887201888) * x_data + (-171.2798264642083) * y_data + (182.4589404436547)
        return x_pixel, y_pixel

    def fetch(self, save_to=None):
        if save_to is None: save_to = self.path
        start = time.time()
        source = urllib2.urlopen(self.address)
        data = source.read()
        delay = time.time() - start
        # print 'Got image in: ', delay
        f = open(save_to, 'wb')
        f.write(data)
        f.close()
        self.image = Image.open(save_to)

    def get_extent(self, rows=640, columns=480):
        max_x, min_y = self.pixel2world(rows, columns)
        min_x, max_y = self.pixel2world(1, 1)
        extent = (min_x, max_x, min_y, max_y)
        return extent

    def plot_arena(self, fetch_new=True, show=False):
        if fetch_new: self.fetch()
        image = pyplot.imread(self.path)
        shape = image.shape
        max_x, min_y = self.pixel2world(shape[1], shape[0])
        min_x, max_y = self.pixel2world(1, 1)
        pyplot.imshow(image, extent=(min_x, max_x, min_y, max_y), origin='upper')
        pyplot.draw()
        if show: pyplot.show()

    def capture(self, folder, rate=1):
        interval = 1.0 / float(rate)
        counter = 0
        while True:
            if self.stop_capture_process: break
            start = time.time()
            counter += 1
            file_name = str(counter).zfill(3) + '.jpg'
            file_name = os.path.join(folder, file_name)
            self.fetch(file_name)
            end = time.time()
            duration = end - start
            rest = interval - duration
            if rest > 0: time.sleep(rest)

    def start_capture(self, folder, rate=1):
        self.stop_capture_process = False
        self.capture_process = threading.Thread(target=self.capture, args=(folder, rate))
        self.capture_process.start()

    def stop_capture(self):
        if self.capture_process.is_alive():
            self.stop_capture_process = True
            self.capture_process.join()


# W = Webcam()
# x = numpy.array([0,0,0])
# print W.world2pixel(x,x)
# W.plot_arena()
# pyplot.hold(True)
# pyplot.plot(0,0,'r+')
# pyplot.plot(0,1,'r+')
# pyplot.hold(False)
# pyplot.show()
