#!/usr/bin/env python
# Copyright (C) 2011-2012 W. Trevor King <wking@tremily.us>
#
# This file is part of pypid.
#
# pypid is free software: you can redistribute it and/or modify it under the
# terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# pypid is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# pypid.  If not, see <http://www.gnu.org/licenses/>.


from argparse import ArgumentParser
from sys import stdout
from time import sleep

try:
    from matplotlib import pyplot
    from numpy import loadtxt
except (ImportError,RuntimeError), e:
    pyplot = None
    loadtxt = None
    plot_import_error = e

from pypid.backend.test import TestBackend
from pypid.rules import ziegler_nichols_step_response


parser = ArgumentParser(description='Simulate a step response.')
parser.add_argument(
    '-K', '--process-gain', metavar='GAIN', type=float, default=1,
    help='process gain (PV-units over MV-units)',)
parser.add_argument(
    '-L', '--dead-time', metavar='TIME', type=float, default=1,
    help='system dead time (lag)')
parser.add_argument(
    '-T', '--decay-time', metavar='TIME', type=float, default=1,
    help='exponential decay timescale')
parser.add_argument(
    '-P', '--proportional', metavar='GAIN', type=float, default=None,
    help='process gain (output units over input units)',)
parser.add_argument(
    '-I', '--integral', metavar='TIME', type=float, default=None,
    help='intergral gain timescale')
parser.add_argument(
    '-D', '--derivative', metavar='TIME', type=float, default=None,
    help='derivative gain timescale')
parser.add_argument(
    '-M', '--max-mv', metavar='MV', type=float, default=100.,
    help='maximum manipulated variable')
parser.add_argument(
    '-A', '--tuning-algorithm', metavar='TUNER', default=None,
    choices=['ZN'], help='step tuning algorithm')
parser.add_argument(
    '-m', '--mode', metavar='MODE', default='PID',
    choices=['P', 'PI', 'PID'], help='controller mode')
parser.add_argument(
    '-t', '--time', metavar='TIME', type=float, default=10.,
    help='simulation time')
parser.add_argument(
    '-o', '--output', default='-', help='output log file')
parser.add_argument(
    '-p', '--plot', action='store_true', default=False,
    help='plot the repsonse')

args = parser.parse_args()

if args.plot and not (pyplot and loadtxt) :
    raise plot_import_error

if args.output == '-':
    log_stream = stdout
    if args.plot:
        raise ValueError('can only plot when outputing to a file')
else:
    log_stream = open(args.output, 'w')

K = args.process_gain
L = args.dead_time
T = args.decay_time

p,i,d = (0, float('inf'), 0)
if args.tuning_algorithm == 'ZN':
    p,i,d = ziegler_nichols_step_response(
        process_gain=K, dead_time=L, decay_time=T, mode=args.mode)
else:
    if args.proportional:
        p = args.proportional
    if args.integral:
        i = args.integral
    if args.derivative:
        d = args.derivative

b = TestBackend(
    process_gain=K, dead_time=L, decay_time=T, max_mv=args.max_mv,
    log_stream=log_stream)
try:
    b.set_up_gains(proportional=p, integral=i, derivative=d)
    b.set_down_gains(proportional=p, integral=i, derivative=d)
    b.set_setpoint(1.0)
    sleep(args.time)
finally:
    b.cleanup();
    if args.output != '-':
        log_stream.close()

if args.plot:
    header = open(args.output, 'r').readline()
    label = header.strip('#\n').split('\t')
    data = loadtxt(args.output)
    times = data[:,0] - data[0,0]
    pyplot.hold(True)
    subplot = 1
    for i in range(1, len(label)):
        if i in [1, 4, 6]:
            if i:
                pyplot.legend(loc='best')  # add legend to previous subplot
            pyplot.subplot(3, 1, subplot)
            subplot += 1
        pyplot.plot(times, data[:,i], '.', label=label[i])
    pyplot.legend(loc='best')
    pyplot.show()
