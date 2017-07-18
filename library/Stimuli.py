__author__ = 'dieter'

import os
import random
import paramiko
import time


def get_connection(username='pi', password='raspberry'):
    server = '164.11.73.19'
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(server, username=username, password=password)
    return ssh

def set_images(ssh,string):
    cmd = 'python ~/Desktop/Remote_Display/set_images.py ' + string
    ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(cmd)
    print ssh_stdin
    print ssh_stdout
    print ssh_stderr

# def start_display_port(connection):
#     chan = connection[1]
#     cmd = 'python ~/Desktop/Remote_Display/display.py'
#     #ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(cmd)
#     time.sleep(3)
#     chan.send(cmd+'\n')
#     #print ssh_stdin
#     #print ssh_stdout
#     #print ssh_stderr
#
#
# def stop_display_port(connection):
#     ssh = connection[0]
#     cmd = 'python ~/Desktop/Remote_Display/set_images.py ' + 'EXIT'
#     ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(cmd)
#     print ssh_stdin
#     print ssh_stdout
#     print ssh_stderr


def filter_list(in_list):
    filtered = []
    for x in in_list:
        if not x.startswith('_'): filtered.append(x)
    return filtered

def get_letters():
    files = os.listdir('Remote_Display/images')
    files = filter_list(files)
    first_letters = []
    for name in files: first_letters.append(name[0])
    first_letters = list(set(first_letters))
    return first_letters


def select(n, collection, sort_seq=True):
    selected = random.sample(collection, n)
    selected.sort()
    if not sort_seq:
        original = selected[:]
        while 1:
            random.shuffle(selected)
            if not selected == original: break
    return selected


def seq2images(seq):
    files = os.listdir('Remote_Display/images')
    files = filter_list(files)
    images = []
    random.shuffle(files)
    for letter in seq:
        for name in files:
            if name.startswith(letter):
                images.append(name)
                break
    return images

def images2str(images):
    string = ''
    for name in images:
        name = name.replace('.jpg', '')
        name = name.replace('.png', '')
        string+=name + ','
    string = string.rstrip(',')
    return string

def make_stimuli_sequence(n, sort=True):
    coll = get_letters()
    seq = select(n, coll, sort)
    images = seq2images(seq)
    string = images2str(images)
    return seq, images, string





