# Copyright (C) 2011 W. Trevor King <wking@drexel.edu>
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

import threading as _threading
import time as _time

from .. import LOG as _LOG
from . import Backend as _Backend
from . import ManualMixin as _ManualMixin
from . import PIDMixin as _PIDMixin


class TestBackend (_Backend, _ManualMixin, _PIDMixin):
    """Test backend for demonstrating `Controller` function

    The underlying temperature decay model is exponential, which is
    often refered to as "Newton's law of cooling"

        dT/dt = h (Tbath - T)

    where `h` is the transfer coefficient (with some scaling terms
    brushed under the rug).  To make the system more realistic, I've
    also added a dead time, so temperatures returned by `.get_temp()`
    actually correspond to the system temperature `dead_time` seconds
    before the measurement was taken.  Finally, there's a
    `drive_coefficient` `d`, which gives the rate of temperature
    change due to a applied driving current `I`, so

        dT(y)/dt = h (Tbath(t) - T(t)) + d I(t-L)

    This model is often refered to as a FOPDT (first-order plus dead
    time) or KLT (K: static gain, L: time delay, T: time constant/lag)
    model.  Translating our model above into process-control jargon:

    * Process variable y(t) corresponds to our T(t).
    * Manipulated variable u(t) corresponds to our I(t).
    * Process gain dy/du (often denoted K_p).  For our parameters
      above, K_p = dy/du = dT/dI = d/h.
    * Process time constant (aka lag, often denoted tau or T; the
      exponential decay timescale).  For our parameters above,
      tau = 1/h.
    * Dead time (often denoted L or theta; the time delay between a
      change system and that change being reflected in the process
      variable).  For our parameters above, L = dead_time.

    The response function for a FOPDT process is

      G(s) = K_p e^{-Ls} / (1 + T s)

    For interesting experimental evidence of exponential cooling, see
    Kaliszan et al., "Verification of the exponential model of body
    temperature decrease after death in pigs".
    doi: 10.1113/expphysiol.2005.030551
    http://ep.physoc.org/content/90/5/727.long
    September 1, 2005 Experimental Physiology, 90, 727-738.
    """
    def __init__(self, bath=20, transfer_coefficient=0.1,
                 drive_coefficient=1., max_current=1., dead_time=1.,
                 process_period=0.01, log_stream=None):
        """
        bath : float
            bath (ambient) temperature in degrees Celsius
        transfer_coefficient : float
            between the system and the bath, in inverse seconds
        drive_coefficient : float
            for the applied current, in degrees Celsius per amp
        max_current : float
            maximum current in amps
        dead_time : float
            time lag in seconds between an internal system temperature
            and the corresponding `.get_temp()` reading
        process_period : float
            time in seconds between process-thread temperature updates
        """
        self._bath = bath
        self._transfer_coefficient = transfer_coefficient
        self._drive_coefficient = drive_coefficient
        self._max_current = max_current
        self._dead_periods = int(dead_time/process_period)
        self._process_period = process_period
        self._log_stream = log_stream
        self._setpoint = 0
        self._i_term = self._d_term = 0
        self._p_cool = self._d_cool = 0
        self._p_heat = self._d_heat = 0
        self._i_cool = self._i_heat = float('inf')
        self._manual_current = 0
        self._mode = 'PID'
        self._temperatures = [bath]*(self._dead_periods+1)
        self._start_process_thread()

    def cleanup(self):
        self._stop_process_thread()

    def _start_process_thread(self):
        self._stop_process = False
        self._process_thread = _threading.Thread(
            target=self._run_process, name='process')
        self._process_thread.start()

    def _stop_process_thread(self):
        self._stop_process = True
        self._process_thread.join()

    def _run_process(self):
        if self._log_stream:
            line = '\t'.join((
                    'time', 'setpoint', 'process temperature',
                    'measured temperature', 'dT_bath', 'dT_drive', 'current',
                    'intergal', 'derivative'))
            self._log_stream.write('#{}\n'.format(line))
        dt = self._process_period
        next_time = _time.time() + dt
        while not self._stop_process:
            T = self._temperatures[-1]
            dT_bath = self._transfer_coefficient * (self._bath - T)
            current = self.get_current(_increment_i_term=True)
            dT_drive = self._drive_coefficient * current
            if self._log_stream:
                line = '\t'.join(str(x) for x in (
                    _time.time(), self._setpoint, T, self.get_temp(), dT_bath*dt,
                    dT_drive*dt, current, self._i_term, self._d_term))
                self._log_stream.write(line + '\n')
            T += (dT_bath + dT_drive) * dt
            self._temperatures.pop(0)
            self._temperatures.append(T)
            s = next_time - _time.time()
            if s > 0:
                _time.sleep(s)
            next_time += dt

    def _limited_current(self, current):
        if current > self._max_current:
            #_LOG.debug('limiting current to maximum: {:n} -> {:n} amps'.format(
            #        current, self._max_current))
            return self._max_current
        elif current < -self._max_current:
            #_LOG.debug('limiting current to maximum: {:n} -> {:n} amps'.format(
            #        current, -self._max_current))
            return -self._max_current
        return current

    def get_temp(self):
        return self._temperatures[1]

    def get_ambient_temp(self):
        return self._bath

    def set_max_current(self, max):
        self._max_current = max

    def get_max_current(self):
        return self._max_current

    def get_current(self, _increment_i_term=True):
        if self._mode == 'manual':
            return self._manual_current
        elif self._mode == 'PID':
            T_pref,T = self._temperatures[:2]
            dT_s = (self._setpoint - T)
            if T > self._setpoint:
                p,i,d = self._p_cool, self._i_cool, self._d_cool
            else:
                p,i,d = self._p_heat, self._i_heat, self._d_heat
            dT_t = T - T_pref
            dt = self._process_period
            if _increment_i_term is True:
                self._i_term += dT_s * dt
            self._d_term = -dT_t / dt  # = de(t)/dt with constant setpoint
            return self._limited_current(
                p*(dT_s + self._i_term/i + d*self._d_term))
        raise ValueError(self._mode)

    def get_modes(self):
        return ['manual', 'PID']

    def get_mode(self):
        return self._mode

    def set_mode(self, mode):
        self._mode = mode

    # ManualMixin methods

    def set_current(self, current):
        self._manual_current = self._limited_current(current)

    # PIDMixin methods

    def set_setpoint(self, setpoint):
        self._setpoint = setpoint

    def get_setpoint(self):
        return self._setpoint

    def set_cooling_gains(self, proportional=None, integral=None,
                          derivative=None):
        if proportional is not None:
            self._p_cool = proportional
        if integral is not None:
            self._i_cool = integral
        if derivative is not None:
            self._d_cool = derivative

    def get_cooling_gains(self):
        return (self._p_cool, self._i_cool, self._d_cool)

    def set_heating_gains(self, proportional=None, integral=None,
                          derivative=None):
        if proportional is not None:
            self._p_heat = proportional
        if integral is not None:
            self._i_heat = integral
        if derivative is not None:
            self._d_heat = derivative

    def get_heating_gains(self):
        return (self._p_heat, self._i_heat, self._d_heat)

    def get_feedback_terms(self):
        return (self.get_current(), self._setpoint - self.get_temp(),
                self._i_term, self._d_term)

    def clear_integral_term(self):
        self._i_term = 0
