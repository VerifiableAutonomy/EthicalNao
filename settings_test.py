#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  3 14:38:18 2017

@author: paul
"""
settings = {}
with open('script_params_template') as f:
    for line in f:
        (key, val) = line.split()
        if ',' in val:
            val = val.split(',')
            for i,v in enumerate(val):
                try:
                    val[i] = float(v)
                except:
                    pass
        else:
            try:
                val = float(val)
            except:
                pass
        settings[key] = val

print settings
