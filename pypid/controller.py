# Copyright (C) 2008-2011 W. Trevor King <wking@drexel.edu>
#
# This file is part of pypid.
#
# pypid is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either
# version 3 of the License, or (at your option) any later version.
#
# pypid is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with pypid.  If not, see
# <http://www.gnu.org/licenses/>.

import time as _time

from aubio.aubioclass import pitch as _pitch
from aubio.aubioclass import fvec as _fvec
from numpy import array as _array
from numpy import exp as _exp
from numpy import linspace as _linspace
from numpy import log as _log
from scipy.interpolate import interp1d as _interp1d

from . import LOG as _LOG
from .fit import ModelFitter as _ModelFitter


class Controller (object):
    """PID control frontend.

    backend: pypid.backend.Backend instance
        backend driving your particular harware
    setpoint: float
        initial setpoint in PV-units
    min: float
        minimum PV in PV-units (for sanity checks)
    max: float
        maximum PV in PV-units (for sanity checks)
    """
    def __init__(self, backend, setpoint=0.0, min=-float('inf'),
                 max=float('inf')):
        self._backend = backend
        self._setpoint = setpoint
        self._min_pv = min
        self._max_pv = max

    # basic user interface methods

    def get_pv(self):
        """Return the current process variable in PV-units

        We should expose this to users, so they don't need to go
        mucking about in `._backend`.
        """
        return self._backend.get_pv()

    def set_pv(self, setpoint, **kwargs):
        """Change setpoint to `setpoint` and wait for stability
        """
        self._backend.set_setpoint(setpoint)
        self.wait_for_stability(setpoint=setpoint, **kwargs)

    def wait_for_stability(self, setpoint, tolerance=0.3, time=10.,
                           timeout=-1, sleep_time=0.1, return_data=False):
        """Wait until the PV is sufficiently stable

        setpoint : float
            target PV in PV-units
        tolerance : float
            maximum allowed deviation from `setpoint` in PV-units
        time : float
            time the PV must remain in the allowed region before the
            signal is delared "stable"
        timeout : float
            maximum time to wait for stability.  Set to -1 to never
            timeout.
        sleep_time : float
            time in seconds to sleep between reads to avoid an
            overly-busy loop
        return_data : boolean
            if true, also return a list of `(timestamp, PV)` tuples
            read while waiting

        Read the PV every `sleep_time` seconds until the PV has
        remained within `tolerance` of `setpoint` for `time`.  If the
        stability criteria are met, return `True` (stable).  If
        `timeout` seconds pass before the criteria are met, return
        `False` (not stable).
        """
        _LOG.debug(('wait until the PV is stable at {:n} +/- {:n} C '
                    'for {:n} seconds').format(setpoint, tolerance, time))
        stable = False
        if return_data:
            data = []
        start_time = _time.time()
        stable_time = start_time + time
        if timeout < 0:
            timeout_time = None
        else:
            timeout_time = start_time + timeout
        while True:
            PV = self.get_pv()
            in_range = abs(PV - setpoint) < tolerance
            t = _time.time()
            if return_data:
                data.append((t, PV))
            if in_range:
                if t >= stable_time:
                    _LOG.debug('process variable is stable')
                    stable = True
                    break  # in range for long enough
            else:
                stable_time = t + time  # reset target time
            if timeout_time and t > timeout_time:
                _LOG.debug('process variable is not stable')
                break  # timeout
            _time.sleep(sleep_time)
        if return_data:
            return (stable, data)
        return stable

    def is_stable(self, setpoint, time, **kwargs):
        return self.wait_for_stability(
            setpoint=setpoint, time=time, timeout=time, **kwargs)

    def estimate_pv_sensitivity(self, values=10, sleep_time=0.1,
                                max_repeats=10):
        PVs = []
        last_PV = None
        repeats = 0
        while True:
            PV = self.get_pv()
            if repeats == max_repeats:
                last_PV = None
            if PV == last_PV:
                repeats += 1
            else:
                PVs.append(PV)
                if len(PVs) > values:
                    break
                repeats = 0
                last_PV = PV
            _time.sleep(sleep_time)
        PVs = _array(PVs)
        return PVs.std()

    # debugging methods

    def check_feedback_terms(self):
        """Check a backend's interpretation of its PID feedback terms.

        Some backends provide an interface to read out their PID
        feedback terms, but the interface is not always well
        documented.  This method reads out the terms, and compares
        them with our own calculations (when possible) to test the
        backend's interpretation.
        """
        c = self._backend.get_current()
        pid,prop,ntgrl,deriv = self._backend.get_feedback_terms()
        PV = self.get_pv()
        SP = self._backend.get_setpoint()
        if PV > SP:
            p,i,d = self._backend.get_down_gains()
        else:
            p,i,d = self._backend.get_up_gains()
        _LOG.info(('pid(read) {:n} =? sum(calc from terms) {:n} '
                   '=? cur(read) {:n} A').format(pid, prop+ntgrl+deriv, c))
        _LOG.info('read: p {:n}, i {:n}, d {:n}'.format(p,i,d))
        _LOG.info('calc: p {:n}'.format(p*(SP-PV)))

    # tuning experiments and data processing

    def get_step_response(self, mv_a, mv_b, sleep_time=0.1, stable_time=10.,
                          **kwargs):
        "Measure a step response for later analysis"
        _LOG.debug('measure step response')
        if 'time' in kwargs:
            raise ValueError(kwargs)
        kwargs['time'] = stable_time
        kwargs['sleep_time'] = sleep_time        
        mode = self._backend.get_mode()
        if mode == 'manual':
            manual_mv = self._backend.get_mv()
        else:
            self._backend.set_mode('manual')
        _LOG.debug('set first current and wait for stability')
        self._backend.set_mv(mv_a)
        pv_a = self.get_pv()
        while not self.is_stable(pv_a, **kwargs):
            pv_a = self.get_pv()
        _LOG.debug('stabilized at {:n} {} with {:n} {}'.format(
                pv_a, self._backend.pv_units, mv_a, self._backend.mv_units))
        _LOG.debug('set second MV and wait for stability')
        data = []
        start_time = _time.time()
        self._backend.set_mv(mv_b)
        pv_b = pv_a
        while True:
            stable,d = self.is_stable(pv_b, return_data=True, **kwargs)
            data.extend(d)
            pv_b = self.get_pv()
            if stable:
                break
        _LOG.debug('stabilized at {:n} {} with {:n} {}'.format(
                pv_b, self._backend.pv_units, mv_b, self._backend.mv_units))
        if mode == 'manual':
            self._backend.set_mv(manual_mv)
        else:
            self._backend.set_mode(mode)
        return data

    @staticmethod
    def analyze_step_response(step_response, mv_shift):
        rates = [(PVb-PVa)/(tb-ta) for ((ta,PVa),(tb,PVb))
                 in zip(step_response, step_response[1:])]
        # TODO: averaging filter?
        max_rate_i = max_rate = 0
        for i,rate in enumerate(rates):
            if abs(rate) > max_rate:  # handle steps in both directions
                max_rate_i = i
                max_rate = abs(rate)
        max_rate_time,max_rate_temp = step_response[max_rate_i]  # TODO: avg i and i+1?
        time_a,PV_a = step_response[0]
        max_rate_time -= time_a
        dead_time = max_rate_time - (max_rate_temp - PV_a) / max_rate
        t_data = _array([t for t,PV in step_response[max_rate_i:]])
        PV_data = _array([PV for t,PV in step_response[max_rate_i:]])
        model = ExponentialModel(PV_data, info={'x data (s)': t_data})
        decay_time,PV0,PV8 = model.fit()
        process_gain = (PV8 - PV_a) / mv_shift
        return (process_gain, dead_time, decay_time, max_rate)

    def get_bang_bang_response(self, dead_band=0.8, num_oscillations=10,
                               max_dead_band_time=30, sleep_time=0.1):
        orig_down_gains = self._backend.get_down_gains()
        orig_up_gains = self._backend.get_up_gains()
        _LOG.debug('measure bang-bang response')
        mode = self._backend.get_mode()
        if mode != 'PID':
            self._backend.set_mode('PID')
        i=0
        setpoint = self._backend.get_setpoint()
        self._backend.set_down_gains(float('inf'), float('inf'), 0)
        self._backend.set_up_gains(float('inf'), float('inf'), 0)
        start_time = _time.time()
        pv = self.get_pv()
        under_first = self._is_under(
            pv=pv, setpoint=setpoint, dead_band=dead_band)
        _LOG.debug('wait to exit dead band')
        t = start_time
        while under_first is None:
            if t - start_time > max_dead_band_time:
                msg = 'still in dead band after after {:n} seconds'.format(
                    max_dead_band_time)
                _LOG.error(msg)
                raise ValueError(msg)
            _time.sleep(sleep_time)
            t = _time.time()
            pv = t.get_pv()
            under_first = self._is_under(
                pv=pv, setpoint=setpoint, dead_band=dead_band)        
        _LOG.debug('read {:d} oscillations'.format(num_oscillations))
        data = []
        under = under_first
        while i < num_oscillations*2 + 1:
            t = _time.time()
            pv = self.get_pv()
            # drop first half cycle (possibly includes ramp to setpoint)
            if i > 0:
                data.append((t, pv))
            _under = self._is_under(
                temp=temp, setpoint=setpoint, dead_band=dead_band)
            if _under is True and under is False:
                _LOG.debug('transition to PV < SP (i={:d})'.format(i))
                under = _under
                i += 1
            elif under is False and is_under is True:
                _LOG.debug('transition to PV > SP (i={:d})'.format(i))
                under = _under
                i += 1
            _time.sleep(sleep_time)
        self._backend.set_down_gains(*orig_down_gains)
        self._backend.set_up_gains(*orig_up_gains)
        if mode != 'PID':
            self._backend.set_mode(mode)
        return data

    @staticmethod
    def analyze_bang_bang_response(bang_bang_response):
        t_data = _array([t for t,PV in bang_bang_response])
        PV_data = _array([PV for t,PV in bang_bang_response])
        amp = (PV_data.max() - PV_data.min()) / 2
        freq = Controller._get_frequency(x_data=t_data, y_data=PV_data)
        period = 1./freq
        return (amp, period)

    def get_ultimate_cycle_response(self, proportional, period):
        orig_down_gains = self._backend.get_down_gains()
        orig_up_gains = self._backend.get_up_gains()
        _LOG.debug('measure ultimate cycle response')
        mode = self._backend.get_mode()
        if mode != 'PID':
            self._backend.set_mode('PID')
        # TODO...
        self._backend.set_down_gains(*orig_down_gains)
        self._backend.set_up_gains(*orig_up_gains)
        if mode != 'PID':
            self._backend.set_mode(mode)
        return data

    @staticmethod
    def analyze_ultimate_cycle_response(ultimate_cycle_response):
        amp,period = Controller.analyze_bang_bang_response(
            ultimate_cycle_response)
        return period

    # utility methods

    def _wait_until_close(self, setpoint, tolerance=0.3, sleep_time=0.1):
        while abs(self.get_pv() - setpoint) > tolerance:
            _time.sleep(sleep_time)

    def _time_function(self, function, args=(), kwargs=None, count=10):
        "Rough estimate timing of get_temp(), takes me about 0.1s"
        if kwargs is None:
            kwargs = {}
        start = _time.time()
        for i in range(count):
            function(*args, **kwargs)
        stop = _time.time()
        return float(stop-start)/count

    def _is_under(self, pv=None, setpoint=None, dead_band=None):
        if pv is None:
            pv = self.get_pv()
        if setpoint is None:
            setpoint = self._backend.get_setpoint()
        low_pv = high_pv = setpoint
        if dead_band:
            low_pv -= dead_band
            high_pv += dead_band
        if pv < low_pv:
            return True
        elif pv > high_pv:
            return False
        return None

    def _select_parameter(self, under_result=None, over_result=None,
                          dead_band_result=None, **kwargs):
        under = self._is_under(**kwargs)
        if under:
            return under_result
        elif under is False:
            return over_result
        return dead_band_result

    @staticmethod
    def _resample_with_constant_dx(x_data, y_data):
        f = _interp1d(x_data, y_data)
        x = _linspace(x_data[0], x_data[-1], len(x_data))
        y = f(x)
        return x, y

    @staticmethod
    def _get_frequency(x_data, y_data):
        x,y = Controller._resample_with_constant_dx(x_data, y_data)        
        dx = x[1] - x[0]
        yvec = _fvec(len(y_data))
        mean = y.mean()
        for i,_y in enumerate(y_data):
            yvec.set(_y - mean, i)
        fake_sample_rate = 8000  # aubio is built for audio
        p = _pitch(mode='schmitt', bufsize=len(y_data), hopsize=len(y_data),
                   samplerate=fake_sample_rate, omode='freq', tolerance=0.1)
        freq = p(yvec) / (fake_sample_rate * dx)
        _LOG.debug('pitch: {:n}, sample rate {:n}'.format(freq, 1./dx))
        del(p)
        del(yvec)
        return freq

    @staticmethod
    def _check_range(value, min, max):
        if value < min:
            raise ValueError('%g < %g' % (value, min))
        if value > max:
            raise ValueError('%g > %g' % (value, max))

    def _check_pv(pv):
        self._check_rangee(pv, self._min_pv, self._max_pv)


class ExponentialModel (_ModelFitter):
    "Exponential decay model"
    def model(self, params):
        tau,y0,y8 = params
        x_data = self.info['x data (s)']
        x0 = x_data[0]  # raw times in seconds are too far from the epoc
        a = 1 - y0/y8
        self._model_data[:] = y8*(1-a*_exp(-(x_data-x0)/tau))
        return self._model_data

    def guess_initial_params(self, outqueue=None, **kwargs):
        x_data = self.info['x data (s)']
        y_data = self._data
        y8 = y_data[-1]
        x_mid = x_data[int(len(x_data)/2)]
        y_mid = y_data[int(len(y_data)/2)]
        x_start = x_data[0]
        y_start = y_data[0]
        tau = (x_mid - x_start)/_log((y_start-y8)/(y_mid-y8))
        return (tau, y_start, y8)

    def guess_scale(self, params, outqueue=None, **kwargs):
        return (1., 1., 1.)
