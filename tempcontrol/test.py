# Copyright (C) 2008-2011 W. Trevor King <wking@drexel.edu>
#
# This file is part of tempcontrol.
#
# tempcontrol is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either
# version 3 of the License, or (at your option) any later version.
#
# tempcontrol is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with tempcontrol.  If not, see
# <http://www.gnu.org/licenses/>.

"Basic testing for `Controller`\s and `Backend`\s"

import time as _time

from . import LOG as _LOG
from .backend import get_backend as _get_backend
from controller import Controller as _Controller


def test_backend(backend=None):
    internal_backend = False
    if not backend:
        internal_backend = True
        backend = _get_backend('test')()
    try:
        sp = backend.get_setpoint()
        _LOG.info('temperature = {:n} C'.format(backend.get_temp()))
        _LOG.info('setpoint    = {:n} C'.format(sp))
        _LOG.info('current     = {:n} A'.format(backend.get_current()))

        _set_and_check_setpoint(backend=backend, setpoint=5.0)
        _check_max_current(backend=backend)
        _set_and_check_setpoint(backend=backend, setpoint=50.0)
        _check_max_current(backend=backend)
        _set_and_check_setpoint(backend=backend, setpoint=sp)
    finally:
        if internal_backend:
            backend.cleanup()

def _set_and_check_setpoint(backend, setpoint):
    _LOG.info('setting setpoint to {:n} C'.format(setpoint))
    c.set_setpoint(setpoint)
    sp = c.get_setpoint()
    _LOG.info('setpoint    = {:n} C'.format(sp))
    if sp != setpoint:
        msg = 'read setpoint {:n} != written setpoint {:n}'.format(
            sp, setpoint)
        _LOG.error(msg)
        raise Exception(msg)

def _check_max_current(backend):
    # give the backend some time to overcome any integral gain
    _time.sleep(10)
    cur = c.get_current()
    _LOG.info('current     = {:n} A'.format(cur))
    mcur = c.get_max_current()
    if cur != mcur:
        temp = backend.get_temp()
        sp = backend.get_setpoint()
        msg = ('current of {:n} A is not the max {:n} A, but the system is '
               'at {:n} C while the setpoint is at {:n}').format(
            cur, mcur, temp, sp)
        _LOG.error(msg)
        raise Exception(msg)

def test_controller_step_response(backend=None, setpoint=25):
    internal_backend = False
    if not backend:
        internal_backend = True
        backend = _get_backend('test')()
    try:
        backend.set_mode('PID')
        c = _Controller(backend=backend)
        max_current = backend.get_max_current()
        current_a = 0.4 * max_current
        current_b = 0.5 * max_current
        step_response = c.get_step_response(
            current_a=current_a, current_b=current_b, tolerance=0.5, stable_time=4.)
        if True:
            with open('step_response.dat', 'w') as d:
                s = step_response[0][0]
                for t,T in step_response:
                    d.write('{:n}\t{:n}\n'.format(t-s, T))
        gain,dead_time,tau,max_rate = c.analyze_step_response(
            step_response, current_shift=current_b-current_a)
        _LOG.debug(('step response: dead time {:n}, gain {:n}, tau {:n}, '
                    'max-rate {:n}').format(dead_time, gain, tau, max_rate))
        for name,response_fn,modes in [
            ('Zeigler-Nichols', c.ziegler_nichols_step_response,
             ['P', 'PI', 'PID']),
            ('Cohen-Coon', c.cohen_coon_step_response,
             ['P', 'PI', 'PID']), # 'PD'
            ('Wang-Juan-Chan', c.wang_juang_chan_step_response,
             ['PID']),
            ]:
            for mode in modes:
                p,i,d = response_fn(
                    gain=gain, dead_time=dead_time, tau=tau, mode=mode)
                _LOG.debug(
                    '{} step response {}: p {:n}, i {:n}, d {:n}'.format(
                        name, mode, p, i, d))
    finally:
        if internal_backend:
            backend.cleanup()

def test_controller_bang_bang_response(backend=None, setpoint=25):
    internal_backend = False
    if not backend:
        internal_backend = True
        backend = _get_backend('test')(log_stream=open('pid.log', 'w'))
        # shift our noise-less system off its setpoint
        backend.set_setpoint(backend.get_temp()+0.1)
    try:
        c = _Controller(backend=backend)
        dead_band = 3*c.estimate_temperature_sensitivity()
        bang_bang_response = c.get_bang_bang_response(dead_band=dead_band, num_oscillations=4)
        if True:
            with open('bang_bang_response.dat', 'w') as d:
                s = bang_bang_response[0][0]
                for t,T in bang_bang_response:
                    d.write('{:n}\t{:n}\n'.format(t-s, T))
        amplitude,period = c.analyze_bang_bang_response(bang_bang_response)
        _LOG.debug('bang-bang response: amplitude {:n}, period {:n}'.format(
                amplitude,period))
        p,i,d = c.ziegler_nichols_bang_bang_response(
            amplitude=amplitude, period=period, mode='PID')
        _LOG.debug(('Zeigler-Nichols bang-bang response: '
                    'p {:n}, i {:n}, d {:n}').format(p, i, d))
    finally:
        if internal_backend:
            backend.cleanup()
