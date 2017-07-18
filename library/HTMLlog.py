# -*- coding: utf-8 -*-
"""
Created on Thu Aug  6 09:43:30 2015

@author: dieter
"""

import HTML
import os


def unique_set(list):
    unique = []
    for x in list:
        if x not in unique:unique.append(x)
    return unique

def serialize(dictionary, depth=0):
    lst = []
    final_depth = depth
    for key, value in dictionary.iteritems():
        if isinstance(value, dict):
            lst.append(['k', depth, key])
            part, final_depth = serialize(value, depth+1)
            if type(part[0]) == list:
                for x in part: lst.append(x)
            else:
                lst.append(part)
        else:
            lst.append(['v', depth, str(key), str(value)])
    return lst, final_depth + 1


def tabulate(dictionary):
    serialized, depth = serialize(dictionary)
    table = []
    for x in serialized:
        line = [''] * (depth + 2)
        node = x[0]
        level = x[1]
        key = x[2]
        if node == 'k':
            line[level] = str(key)
        if node == 'v':
            line[level] = str(key)
            line[level + 1] = str(x[3])
        table.append(line)
    return table


def dict2table(dictionary, title=False):
    html_code = ''
    if title: html_code = '<h1>' + title + '</h1>'
    table = tabulate(dictionary)
    html_code += HTML.table(table)
    return html_code


def log2html(file_name):
    f = open(file_name, 'r')
    lines = f.readlines()
    f.close()
    # HTML_COLORS = ['#a6cee3','#1f78b4','#b2df8a','#33a02c','#fb9a99','#e31a1c','#fdbf6f','#ff7f00','#cab2d6','#6a3d9a','#ffff99','#b15928']
    html_colors = ['#8dd3c7', '#ffffb3', '#bebada', '#fb8072', '#80b1d3', '#fdb462', '#b3de69', '#fccde5', '#d9d9d9',
                   '#bc80bd', '#ccebc5', '#ffed6f']
    table = HTML.Table(header_row=['Time', 'Agent', 'Message'])
    agents = []
    for line in lines:
        line = line.rstrip('\n')
        line = line.split(';')
        agent = line[1]
        agents.append(agent)
        agent_set = unique_set(agents)
        index = agent_set.index(agent)
        row = HTML.TableRow(line, bgcolor=html_colors[index])
        table.rows.append(row)
    html_code = str(table)
    file_name = os.path.splitext(file_name)
    file_name = file_name[0] + '.html'
    f = open(file_name, 'w')
    f.write(html_code)
    f.close()

# htmlcode = dict2table(test)

# f = open('test.html','w')
# f.write(htmlcode)
# f.close()
