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

import threading as _threading
import time as _time

from .. import LOG as _LOG
from . import Backend as _Backend
from . import ManualMixin as _ManualMixin
from . import PIDMixin as _PIDMixin


class TestBackend (_Backend, _ManualMixin, _PIDMixin):
    """Test backend for demonstrating `Controller` function

    The response function for a first-order plus dead time process
    (aka FOPDT, or occasionaly KLT), one of control theory's classics,
    is

      G(s) = Y(s)/X(s) = K_p e^{-Ls} / (1 + T s)

    Going back to the time domain with the inverse Laplace transform:

      (1 + Ts)Y(s) = K_p e^{-Ls} X(s)              Laplace transform rule:
      y(t) + T dy(t)/dt = K_p e^{-Ls} X(s)         f'(t) -> sF(s) + f(0)
      y(t) + T dy(t)/dt = K_p x(t-L)u(t-L)         f(t-a)u(t-a) -> e^{-as}F(s)
      dy(t)/dt = -y(t)/T + K_p/T x(t-L)u(t-L)

    This may look more familiar to those of us more used to
    differential equations than we are to transfer functions.  There
    is an exponential decay to y=0 with a time constant T, with an
    external driver that has a gain of K_p and a lag (dead time) of L.

    The units should be:

    * K_p: y-units over x-units (e.g. K/amp for a thermoelectric controller)
    * L, T: time (e.g. seconds)

    Just to flesh out the usual jargon, x is refered to as the
    manipulated variable (MV) and y is the process variable (PV).

    Because transfer functions treats the initial conditions
    (e.g. x(t=0)), we can add them back in if we want to create a more
    general y(t) having the same dynamics.

      dy(t)/dt = (y_b(t) - y(t))/T + K_p/T x(t-L)

    where I've assumed that x(t<0)=0 in order to drop the Heaviside
    step function.  Now instead of decaying to zero in the absence of
    a driving signal, y(t) will decay to a bath value y_b(t).

    For interesting experimental evidence of exponential cooling, see
    Kaliszan et al., "Verification of the exponential model of body
    temperature decrease after death in pigs".
    doi: 10.1113/expphysiol.2005.030551
    http://ep.physoc.org/content/90/5/727.long
    September 1, 2005 Experimental Physiology, 90, 727-738.
    """
    def __init__(self, bath=0., process_gain=1., dead_time=1.,
                 decay_time=1., max_mv=1., process_period=0.01,
                 log_stream=None):
        """
        bath : float
            bath (ambient) process variable in PV units
        process_gain : float
            gain between manipulated variable MV and PV, in PV-units/MV-units
        dead_time : float
            time lag between a PV and the corresponding MV response.
        decay_time : float
            exponential decay time constant for the undriven PV
        max_mv : float
            maximum MV in MV-units
        process_period : float
            time between process-thread PV updates

        All times (`dead_time`, `decay_time`, and `process_period`)
        should be in the same units (e.g. seconds).
        """
        self._bath = self._setpoint = bath
        self._K = process_gain
        self._L = dead_time
        self._T = decay_time
        self._max_mv = max_mv
        self._dead_periods = int(dead_time/process_period)
        self._process_period = process_period
        self._log_stream = log_stream
        self._i_term = self._d_term = 0
        self._p_down = self._d_down = 0
        self._p_up = self._d_up = 0
        self._i_down = self._i_up = float('inf')
        self._manual_mv = 0
        self._mode = 'PID'
        self._PVs = [bath]*(self._dead_periods+1)
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
                    'time', 'SP', 'PV_int', 'PV', 'dPV_bath', 'dPV_drive',
                    'MV', 'intergal', 'derivative'))
            self._log_stream.write('#{}\n'.format(line))
        dt = self._process_period
        next_time = _time.time() + dt
        while not self._stop_process:
            PV = self._PVs[-1]
            dPV_bath = (self._bath - PV)/self._T * dt
            MV = self.get_mv(_increment_i_term=True)
            dPV_drive = self._K * MV / self._T * dt
            if self._log_stream:
                line = '\t'.join(str(x) for x in (
                    _time.time(), self._setpoint, PV, self.get_pv(),
                    dPV_bath, dPV_drive, MV, self._i_term, self._d_term))
                self._log_stream.write(line + '\n')
            PV += dPV_bath + dPV_drive
            self._PVs.pop(0)
            self._PVs.append(PV)
            s = next_time - _time.time()
            if s > 0:
                _time.sleep(s)
            next_time += dt

    def _limited_mv(self, mv):
        if mv > self._max_mv:
            #_LOG.debug('limiting mv to maximum: {:n} -> {:n} amps'.format(
            #        mv, self._max_mv))
            return self._max_mv
        elif mv < -self._max_mv:
            #_LOG.debug('limiting mv to maximum: {:n} -> {:n} amps'.format(
            #        mv, -self._max_mv))
            return -self._max_mv
        return mv

    def get_pv(self):
        return self._PVs[1]

    def get_ambient_pv(self):
        return self._bath

    def set_max_mv(self, max):
        self._max_mv = max

    def get_max_mv(self):
        return self._max_mv

    def get_mv(self, _increment_i_term=True):
        if self._mode == 'manual':
            return self._manual_mv
        elif self._mode == 'PID':
            PV_prev,PV = self._PVs[:2]
            dPV_s = (self._setpoint - PV)
            if PV > self._setpoint:
                p,i,d = self._p_down, self._i_down, self._d_down
            else:
                p,i,d = self._p_up, self._i_up, self._d_up
            dPV_t = PV - PV_prev
            dt = self._process_period
            if _increment_i_term is True:
                self._i_term += dPV_s * dt
            self._d_term = -dPV_t / dt  # = de(t)/dt with constant setpoint
            return self._limited_mv(
                p*(dPV_s + self._i_term/i + d*self._d_term))
        raise ValueError(self._mode)

    def get_modes(self):
        return ['manual', 'PID']

    def get_mode(self):
        return self._mode

    def set_mode(self, mode):
        self._mode = mode

    # ManualMixin methods

    def set_mv(self, mv):
        self._manual_mv = self._limited_mv(mv)

    # PIDMixin methods

    def set_setpoint(self, setpoint):
        self._setpoint = setpoint

    def get_setpoint(self):
        return self._setpoint

    def set_down_gains(self, proportional=None, integral=None,
                          derivative=None):
        if proportional is not None:
            self._p_down = proportional
        if integral is not None:
            self._i_down = integral
        if derivative is not None:
            self._d_down = derivative

    def get_down_gains(self):
        return (self._p_down, self._i_down, self._d_down)

    def set_up_gains(self, proportional=None, integral=None,
                          derivative=None):
        if proportional is not None:
            self._p_up = proportional
        if integral is not None:
            self._i_up = integral
        if derivative is not None:
            self._d_up = derivative

    def get_up_gains(self):
        return (self._p_up, self._i_up, self._d_up)

    def get_feedback_terms(self):
        return (self.get_mv(), self._setpoint - self.get_pv(),
                self._i_term, self._d_term)

    def clear_integral_term(self):
        self._i_term = 0
