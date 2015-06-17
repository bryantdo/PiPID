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

"Basic testing for `Controller`\s and `Backend`\s"

import time as _time

from . import LOG as _LOG
from . import rules as _rules
from .backend import get_backend as _get_backend
from controller import Controller as _Controller


def test_backend(backend=None):
    internal_backend = False
    if not backend:
        internal_backend = True
        backend = _get_backend('test')()
    try:
        sp = backend.get_setpoint()
        _LOG.info('PV = {:n} {}'.format(backend.get_pv(), backend.pv_units))
        _LOG.info('SP = {:n} {}'.format(sp, backend.pv_units))
        _LOG.info('MV = {:n} {}'.format(backend.get_mv(), backend.mv_units))

        _set_and_check_setpoint(backend=backend, setpoint=5.0)
        _check_max_mv(backend=backend)
        _set_and_check_setpoint(backend=backend, setpoint=50.0)
        _check_max_mv(backend=backend)
        _set_and_check_setpoint(backend=backend, setpoint=sp)
    finally:
        if internal_backend:
            backend.cleanup()

def _set_and_check_setpoint(backend, setpoint):
    _LOG.info('setting setpoint to {:n} {}'.format(setpoint, backend.pv_units))
    c.set_setpoint(setpoint)
    sp = c.get_setpoint()
    _LOG.info('SP = {:n} {}'.format(sp, backend.pv_units))
    if sp != setpoint:
        msg = 'read setpoint {:n} != written setpoint {:n}'.format(
            sp, setpoint)
        _LOG.error(msg)
        raise Exception(msg)

def _check_max_current(backend):
    # give the backend some time to overcome any integral gain
    _time.sleep(10)
    MV = c.get_mv()
    _LOG.info('MV = {:n} {}'.format(MV, backend.mv_units))
    mMV = c.get_max_mv()
    if mv != mMV:
        PV = backend.get_pv()
        SP = backend.get_setpoint()
        PVu = backend.pv_units
        MVu = backend.mv_units
        msg = ('current of {:n} {} is not the max {:n} {}, but the system is '
               'at {:n} {} while the setpoint is at {:n} {}').format(
            MV, MVu, mMV, MVu, PV, PVu, SP, PVu)
        _LOG.error(msg)
        raise Exception(msg)

def test_controller_step_response(backend=None, setpoint=1):
    internal_backend = False
    if not backend:
        internal_backend = True
        backend = _get_backend('test')()
    try:
        backend.set_mode('PID')
        c = _Controller(backend=backend)
        max_MV = backend.get_max_mv()
        MVa = 0.4 * max_MV
        MVb = 0.5 * max_MV
        step_response = c.get_step_response(
            mv_a=MVa, mv_b=MVb, tolerance=0.5, stable_time=4.)
        if False:
            with open('step_response.dat', 'w') as d:
                s = step_response[0][0]
                for t,PV in step_response:
                    d.write('{:n}\t{:n}\n'.format(t-s, PV))
        process_gain,dead_time,decay_time,max_rate = c.analyze_step_response(
            step_response, mv_shift=MVb-MVa)
        _LOG.debug(('step response: process gain {:n}, dead time {:n}, decay '
                    'time {:n}, max-rate {:n}').format(
                process_gain, dead_time, decay_time, max_rate))
        r = rules
        for name,response_fn,modes in [
            ('Zeigler-Nichols', r.ziegler_nichols_step_response,
             ['P', 'PI', 'PID']),
            ('Cohen-Coon', r.cohen_coon_step_response,
             ['P', 'PI', 'PID']), # 'PD'
            ('Wang-Juan-Chan', r.wang_juang_chan_step_response,
             ['PID']),
            ]:
            for mode in modes:
                p,i,d = response_fn(
                    process_gain=process_gain, dead_time=dead_time,
                    decay_time=decay_time, mode=mode)
                _LOG.debug(
                    '{} step response {}: p {:n}, i {:n}, d {:n}'.format(
                        name, mode, p, i, d))
    finally:
        if internal_backend:
            backend.cleanup()

def test_controller_bang_bang_response(backend=None, setpoint=1):
    internal_backend = False
    if not backend:
        internal_backend = True
        backend = _get_backend('test')()
    try:
        backend.set_setpoint(setpoint)
        c = _Controller(backend=backend)
        dead_band = 3*c.estimate_pv_sensitivity()
        bang_bang_response = c.get_bang_bang_response(dead_band=dead_band)
        if False:
            with open('bang_bang_response.dat', 'w') as d:
                s = bang_bang_response[0][0]
                for t,T in bang_bang_response:
                    d.write('{:n}\t{:n}\n'.format(t-s, T))
        amplitude,period = c.analyze_bang_bang_response(bang_bang_response)
        _LOG.debug('bang-bang response: amplitude {:n}, period {:n}'.format(
                amplitude,period))
        for name,response_fn,modes in [
            ('Zeigler-Nichols', r.ziegler_nichols_bang_bang_response,
             ['P', 'PI', 'PID']),
            ]:
            for mode in modes:
                p,i,d = response_fn(
                    amplitude=amplitude, period=period, mode=mode)
                _LOG.debug(
                    '{} bang-bang response {}: p {:n}, i {:n}, d {:n}'.format(
                        name, mode, p, i, d))
    finally:
        if internal_backend:
            backend.cleanup()
