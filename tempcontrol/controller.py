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

import time as _time

from aubio.aubioclass import pitch as _pitch
from aubio.aubioclass import fvec as _fvec
from numpy import array as _array
from numpy import exp as _exp
from numpy import linspace as _linspace
from numpy import log as _log
from scipy.interpolate import interp1d as _interp1d

from hooke.util.fit import ModelFitter as _ModelFitter

from . import LOG as _LOG


class Controller (object):
    """PID temperature control frontend.

    backend: tempcontrol.backend.Backend instance
        backend driving your particular harware
    setpoint: float
        initial setpoint in degrees Celsius
    min: float
        minimum temperature in degrees Celsius (for sanity checks)
    max: float
        maximum temperature in degrees Celsius (for sanity checks)
    """
    def __init__(self, backend, setpoint=20.0, min=5.0, max=50.0):
        self._backend = backend
        self._setpoint = setpoint
        self._min = min
        self._max = max

    # basic user interface methods

    def get_temp(self):
        """Return the current process temperature in degrees Celsius

        We should expose this to users, so they don't need to go
        mucking about in `._backend`.
        """
        return self._backend.get_temp()

    def set_temp(self, setpoint, **kwargs):
        """Change setpoint to `setpoint` and wait for stability
        """
        self._backend.set_setpoint(setpoint)
        self.wait_for_stability(setpoint=setpoint, **kwargs)

    def wait_for_stability(self, setpoint, tolerance=0.3, time=10.,
                           timeout=-1, sleep_time=0.1, return_data=False):
        """Wait until the temperature is sufficiently stable

        setpoint : float
            target temperature in degrees C
        tolerance : float
            maximum allowed deviation from `setpoint` in dregrees C
        time : float
            time the temperature must remain in the allowed region
            before the signal is delared "stable"
        timeout : float
            maximum time to wait for stability.  Set to -1 to never
            timeout.
        sleep_time : float
            time in seconds to sleep between reads to avoid an
            overly-busy loop
        return_data : boolean
            if true, also return a list of `(timestamp, temp)` tuples
            read while waiting

        Read the temperature every `sleep_time` seconds until the
        temperature has remained within `tolerance` of `setpoint` for
        `time`.  If the stability criteria are met, return `True`
        (stable).  If `timeout` seconds pass before the criteria are
        met, return `False` (not stable).
        """
        _LOG.debug(('wait until the temperature is stable at {:n} +/- {:n} C '
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
            T = self.get_temp()
            in_range = abs(T - setpoint) < tolerance
            t = _time.time()
            if return_data:
                data.append((t, T))
            if in_range:
                if t >= stable_time:
                    _LOG.debug('temperature is stable')
                    stable = True
                    break  # in range for long enough
            else:
                stable_time = t + time  # reset target time
            if timeout_time and t > timeout_time:
                break  # timeout
            _time.sleep(sleep_time)
        if return_data:
            return (stable, data)
        return stable

    def is_stable(self, setpoint, time, **kwargs):
        return self.wait_for_stability(
            setpoint=setpoint, time=time, timeout=time, **kwargs)

    def estimate_temperature_sensitivity(self, num_temps=10, sleep_time=0.1,
                                         max_repeats=10):
        temps = []
        last_temp = None
        repeats = 0
        while True:
            temp = self.get_temp()
            if repeats == max_repeats:
                last_temp = None
            if temp == last_temp:
                repeats += 1
            else:
                temps.append(temp)
                if len(temps) > num_temps:
                    break
                repeats = 0
                last_temp = temp
            _time.sleep(sleep_time)
        temps = _array(temps)
        return temps.std()

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
        T = self.get_temp()
        Tset = self._backend.get_setpoint()
        if T > Tset:  # cooling
            p,i,d = self._backend.get_cooling_gains()
        else:  # heating
            p,i,d = self._backend.get_heating_gains()
        _LOG.info(('pid(read) {:n} =? sum(calc from terms) {:n} '
                   '=? cur(read) {:n} A').format(pid, prop+ntgrl+deriv, c))
        _LOG.info('read: p {:n}, i {:n}, d {:n}'.format(p,i,d))
        _LOG.info('calc: p {:n}'.format(p*(Tset-T)))

    # tuning experiments and data processing

    def get_step_response(self, current_a, current_b,
                          sleep_time=0.1, stable_time=10., **kwargs):
        "Measure a step response for later analysis"
        _LOG.debug('measure step response')
        if 'time' in kwargs:
            raise ValueError(kwargs)
        kwargs['time'] = stable_time
        kwargs['sleep_time'] = sleep_time        
        mode = self._backend.get_mode()
        if mode == 'manual':
            manual_current = self._backend.get_current()
        else:
            self._backend.set_mode('manual')
        _LOG.debug('set first current and wait for stability')
        self._backend.set_current(current_a)
        temp_a = self.get_temp()
        while not self.is_stable(temp_a, **kwargs):
            temp_a = self.get_temp()
        _LOG.debug('stabilized at {:n} C with {:n} amps'.format(
                temp_a, current_a))
        _LOG.debug('set second current and wait for stability')
        data = []
        start_time = _time.time()
        self._backend.set_current(current_b)
        temp_b = temp_a
        while True:
            stable,d = self.is_stable(temp_b, return_data=True, **kwargs)
            data.extend(d)
            temp_b = self.get_temp()
            if stable:
                break
        _LOG.debug('stabilized at {:n} C with {:n} amps'.format(
                temp_b, current_b))
        if mode == 'manual':
            self._backend.set_current(manual_current)
        else:
            self._backend.set_mode(mode)
        return data

    @staticmethod
    def analyze_step_response(step_response, current_shift):
        rates = [(Tb-Ta)/(tb-ta) for ((ta,Ta),(tb,Tb))
                 in zip(step_response, step_response[1:])]
        # TODO: averaging filter?
        max_rate_i = max_rate = 0
        for i,rate in enumerate(rates):
            if abs(rate) > max_rate:  # handle steps in both directions
                max_rate_i = i
                max_rate = abs(rate)
        max_rate_time,max_rate_temp = step_response[max_rate_i]  # TODO: avg i and i+1?
        time_a,temp_a = step_response[0]
        max_rate_time -= time_a
        dead_time = max_rate_time - (max_rate_temp - temp_a) / max_rate
        t_data = _array([t for t,T in step_response[max_rate_i:]])
        T_data = _array([T for t,T in step_response[max_rate_i:]])
        model = ExponentialModel(T_data, info={'x data (s)': t_data})
        tau,T0,T8 = model.fit()
        gain = (T8 - temp_a) / current_shift
        return (gain, dead_time, tau, max_rate)

    def get_bang_bang_response(self, dead_band=0.8, num_oscillations=10,
                               max_dead_band_time=30, sleep_time=0.1):
        orig_cool_gains = self._backend.get_cooling_gains()
        orig_heat_gains = self._backend.get_heating_gains()
        _LOG.debug('measure bang-bang response')
        mode = self._backend.get_mode()
        if mode != 'PID':
            self._backend.set_mode('PID')
        i=0
        setpoint = self._backend.get_setpoint()
        self._backend.set_cooling_gains(float('inf'), float('inf'), 0)
        self._backend.set_heating_gains(float('inf'), float('inf'), 0)
        start_time = _time.time()
        temp = self.get_temp()
        heat_first = self._is_heating(
            temp=temp, setpoint=setpoint, dead_band=dead_band)
        _LOG.debug('wait to exit dead band')
        t = start_time
        while heat_first is None:
            if t - start_time > max_dead_band_time:
                msg = 'still in dead band after after {:n} seconds'.format(
                    max_dead_band_time)
                _LOG.error(msg)
                raise ValueError(msg)
            _time.sleep(sleep_time)
            t = _time.time()
            temp = t.get_temp()
            heat_first = self._is_heating(
                temp=temp, setpoint=setpoint, dead_band=dead_band)        
        _LOG.debug('read {:d} oscillations'.format(num_oscillations))
        data = []
        heating = heat_first
        while i < num_oscillations*2 + 1:
            t = _time.time()
            temp = self.get_temp()
            # drop first half cycle (possibly includes ramp to setpoint)
            if i > 0:
                data.append((t, temp))
            is_heating = self._is_heating(
                temp=temp, setpoint=setpoint, dead_band=dead_band)
            if heating is True and is_heating is False:
                _LOG.debug('transition to cooling (i={:d})'.format(i))
                heating = False
                i += 1
            elif heating is False and is_heating is True:
                _LOG.debug('transition to heating (i={:d})'.format(i))
                heating = True
                i += 1
            _time.sleep(sleep_time)
        self._backend.set_cooling_gains(*orig_cool_gains)
        self._backend.set_heating_gains(*orig_heat_gains)
        if mode != 'PID':
            self._backend.set_mode(mode)
        return data

    @staticmethod
    def analyze_bang_bang_response(bang_bang_response):
        t_data = _array([t for t,T in bang_bang_response])
        T_data = _array([T for t,T in bang_bang_response])
        amp = (T_data.max() - T_data.min()) / 2
        freq = Controller._get_frequency(x_data=t_data, y_data=T_data)
        period = 1./freq
        return (amp, period)

    def get_ultimate_cycle_response(self, proportional, period):
        orig_cool_gains = self._backend.get_cooling_gains()
        orig_heat_gains = self._backend.get_heating_gains()
        _LOG.debug('measure ultimate cycle response')
        mode = self._backend.get_mode()
        if mode != 'PID':
            self._backend.set_mode('PID')
        # TODO...
        self._backend.set_cooling_gains(*orig_cool_gains)
        self._backend.set_heating_gains(*orig_heat_gains)
        if mode != 'PID':
            self._backend.set_mode(mode)
        return data

    @staticmethod
    def analyze_ultimate_cycle_response(ultimate_cycle_response):
        amp,period = Controller.analyze_bang_bang_response(
            ultimate_cycle_response)
        return period

    # tuning rules

    @staticmethod
    def ziegler_nichols_step_response(gain, dead_time, tau, mode='PID'):
        r = dead_time / tau
        if r < 0.1 or r > 1:
            _LOG.warn(('Ziegler-Nichols not a good idea when '
                       'dead-time/tau = {:n}').format(r))
        pkern = tau/(gain*dead_time)
        if mode == 'P':
            return (pkern, float('inf'), 0)
        elif mode == 'PI':
            return (0.9*pkern, 3.3*dead_time, 0)
        elif mode == 'PID':
            return (1.2*pkern, 2*dead_time, dead_time/2.)
        raise ValueError(mode)

    def ziegler_nichols_bang_bang_response(self, amplitude, period,
                                           max_current=None, mode='PID'):
        if max_current is None:
            max_current = self._backend.get_max_current()
        return self._ziegler_nichols_bang_bang_response(
            amplitude, period, max_current=max_current, mode=mode)

    @staticmethod
    def _ziegler_nichols_bang_bang_response(amplitude, period,
                                            max_current, mode='PID'):
        """
        amplitude : float
            center-to-peak amplitude (in K) of bang-bang oscillation
        period : float
            period (in seconds) of the critical oscillation
        max_current : float
            "bang" current (in amps)
        """
        proportional = float(max_current)/amplitude
        period = float(period)
        if mode == 'P':
            return (proportional/2, float('inf'), 0)
        elif mode == 'PI':
            return (proportional/3, 2*period, 0)
        elif mode == 'PID':
            return (proportional/2, period, period/4)
        raise ValueError(mode)

    def ziegler_nichols_ultimate_cycle_response(self, proportional, period):
        """
        proportional : float
            critical P-only gain (ultimate gain, for sustained oscillation)
        period : float
            period (in seconds) of the critical oscillation

        Microstar Laboratories has a `nice analysis`_ on ZN
        limitations, which points out that ZN-tuning assumes your
        system has the FOPDT transfer function (see `TestBackend` for
        details).

        .. _nice analysis: http://www.mstarlabs.com/control/znrule.html
        """
        if mode == 'P':
            return (0.50*proportional, float('inf'), 0)
        elif mode == 'PI':
            return (0.45*proportional, period/1.2, 0)
        elif mode == 'PID':
            return (0.60*proportional, period/2, period/8)
        raise ValueError(mode)

    @staticmethod
    def cohen_coon_step_response(gain, dead_time, tau, mode='PID'):
        r = dead_time / tau
        pkern = tau/(gain*dead_time)
        if mode == 'P':
            return (pkern*(1+r/3.), float('inf'), 0)
        elif mode == 'PI':
            return (pkern*(0.9+r/12.), (30.+3*r)/(9+20*r)*dead_time, 0)
        elif mode == 'PD':  # double check
            return (1.24*pkern*(1+0.13*tf), float('inf'),
                    (0.27-0.36*t)/(1-0.87*t)*dead_time)
        elif mode == 'PID':
            return (pkern*(4./3+r/4.), (32.-6*r)/(13.-8*r)*dead_time,
                    4/(11.+2*r)*dead_time)
        raise ValueError(mode)

    @staticmethod
    def wang_juang_chan_step_response(gain, dead_time, tau, mode='PID'):
        """Wang-Juang-Chan tuning
        """
        K,L,T = (gain, dead_time, tau)
        if mode == 'PID':
            return ((0.7303+0.5307*T/L)*(T+0.5*L)/(K*(T+L)),
                    T + 0.5*L,
                    0.5*L*T / (T + 0.5*L))
        raise ValueError(mode)        

    # utility methods

    def _wait_until_close(self, setpoint, tolerance=0.3, sleep_time=0.1):
        while abs(self.get_temp() - setpoint) > tolerance:
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

    def _is_heating(self, temp=None, setpoint=None, dead_band=None):
        if temp is None:
            temp = self.get_temp()
        if setpoint is None:
            temp = self._backend.get_setpoint()
        low_temp = high_temp = setpoint
        if dead_band:
            low_temp -= dead_band
            high_temp += dead_band
        if temp < low_temp:
            return False
        elif temp > high_temp:
            return True
        return None

    def _select_parameter(self, heating_result=None, cooling_result=None,
                          dead_band_result=None, **kwargs):
        heating = self._is_heating(**kwargs)
        if heating:
            return heating_result
        elif heating is False:
            return cooling_result
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

    def _check_temp(temp):
        self._check_range(temp, self._min, self._max)


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
