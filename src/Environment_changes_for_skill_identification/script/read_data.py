#!/usr/bin/env python

import numpy as np



if __name__ == '__main__':

	 
    
	x, y, z =numpy.loadtxt('topic5.txt', delimiter=',', usecols = (12,13,14), unpack=True)
	x,y,z = numpy.genfromtxt('topic5.txt', dtype=<class 'float'>, comments='#', delimiter=',', skip_header=0, skip_footer=0, converters=None, missing_values=None, filling_values=None, usecols=(12,13,14), names=None, excludelist=None, deletechars=" !#$%&'()*+, -./:;<=>?@[\\]^{|}~", replace_space='_', autostrip=False, case_sensitive=True, defaultfmt='f%i', unpack=True, usemask=False, loose=True, invalid_raise=True, max_rows=None, encoding='bytes', *, like=None)
