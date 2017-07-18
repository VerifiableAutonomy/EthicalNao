from matplotlib import cm
import numpy


def read_palette(file_name):
    f = open(file_name, 'r')
    txt = f.readlines()
    f.close()
    colors = {}
    for x in txt:
        x = x.replace(' ', '')
        x = x.replace('\n', '')
        x = x.replace('~', '')
        x = x.split(',')
        color = [float(x[0]), float(x[1]), float(x[2])]
        name = str(x[3])
        colors[name] = color
    return colors


def color_brewer(colormap, n):
    color_map = cm.get_cmap(name=colormap)
    my_colors = [color_map(i) for i in numpy.linspace(0, 1, n)]
    return my_colors
