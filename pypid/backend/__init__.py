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

"""Assorted backends for interfacing with your particular hardware.
"""


def _import_by_name(modname):
    """
    >>> mod = _import_by_name('pypid.backend.melcor')
    >>> 'MelcorBackend' in dir(mod)
    True
    >>> _import_by_name('pypid.backend.highly_unlikely')
    Traceback (most recent call last):
      ...
    ImportError: No module named highly_unlikely
    """
    module = __import__(modname)
    components = modname.split('.')
    for comp in components[1:]:
        module = getattr(module, comp)
    return module

def get_backend(name):
    n = '%s.%s' % (__name__, name)
    mod = _import_by_name(n)
    for attr in dir(mod):
        obj = getattr(mod, attr)
        try:
            if obj != Backend and issubclass(obj, Backend):
                return obj
        except TypeError:
            pass
    raise ValueError(name)


class Backend (object):
    """Temperature control backend

    There are several common forms for a PID control formula.  For the
    purpose of setting heating and cooling gains (`.get_*_gains()` and
    `.set_*_gains()`), we'll use the standard form::

      MV(t) = K_p ( e(t) + 1/T_i \int_0^t e(\tau) d\tau + T_d de(t)/dt )

    where `e(t) = SP - PV` is the error function, MV is the
    manipulated variable, SP is the setpoint, and PV is the process
    variable.

    In this formulation, the parameter units will be:

    * K_p:  MV units / PV units  (e.g. amp/K)
    * T_i, T_d:  time  (e.g. seconds)
    """
    def __init__(self):
        self._max_current = None

    @staticmethod
    def _convert_F_to_C(F):
        return (F - 32)/1.8

    @staticmethod
    def _convert_C_to_F(C):
        return C*1.8 + 32

    def cleanup(self):
        "Release resources and disconnect from any hardware."
        pass

    def get_temp(self):
        "Return the current process temperature in degrees Celsius"
        raise NotImplementedError()

    def get_ambient_temp(self):
        "Return room temperature in degrees Celsius"
        raise NotImplementedError()

    def set_max_current(self, max):
        "Set the max current in Amps"
        raise NotImplementedError()

    def get_max_current(self):
        "Get the max current in Amps"
        raise NotImplementedError()

    def get_current(self):
        """Return the calculated control current in Amps"

        The returned current is not the actual current, but the
        current that the temperature controller calculates it should
        generate.  If the voltage required to generate that current
        exceeds the controller's max voltage (15 V on mine), then the
        physical current will be less than the value returned here.
        """
        raise NotImplementedError()

    def get_modes(self):
        "Return a list of control modes supported by this backend"
        raise NotImplementedError()

    def get_mode(self):
        "Return the current control mode"
        raise NotImplementedError()

    def set_mode(self, mode):
        "Set the current control mode"
        raise NotImplementedError

    def dump_configuration(self):
        """
        """
        raise NotImplementedError()

    def restore_configuration(self):
        """
        """
        raise NotImplementedError()


class ManualMixin (object):
    def set_current(self, current):
        """Set the desired control current in Amps
        """
        raise NotImplementedError()


class PIDMixin (object):
    def set_setpoint(self, setpoint):
        "Set the temperature setpoint in degrees Celsius"
        raise NotImplementedError()
        
    def get_setpoint(self, setpoint):
        "Get the temperature setpoint in degrees Celsius"
        raise NotImplementedError()

    def get_cooling_gains(self):
        """..."""
        raise NotImplementedError()

    def set_cooling_gains(self, proportional=None, integral=None,
                          derivative=None):
        """
        ...
        """
        raise NotImplementedError()

    def get_heating_gains(self):
        """..."""
        raise NotImplementedError()

    def set_heating_gains(self, proportional=None, integral=None,
                          derivative=None):
        """
        ...
        """
        raise NotImplementedError()

    def get_feedback_terms(self):
        """Experimental
        """
        raise NotImplementedError()

    def clear_integral_term(self):
        """Reset the integral feedback turn (removing integrator windup)

        Because the proportional term provides no control signal when
        the system exactly matches the setpoint, a P-only algorithm
        will tend to "droop" off the setpoint.  The equlibrium
        position is one where the droop-generated P term balances the
        systems temperature leakage.  To correct for this, we add the
        integral feedback term, which adjusts the control signal to
        minimize long-term differences between the output and setpoint.

        One issue with the integral term is "integral windup".  When
        the signal spends a significant time away from the setpoint
        (e.g. during a long ramp up to operating temperature), the
        integral term can grow very large, causing overshoot once the
        output reaches the setpoint.  To allow our controller to avoid
        this, this method manually clears the intergal term for the
        backend.
        """
        raise NotImplementedError()
