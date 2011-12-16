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
    """Process-control backend

    There are several common forms for a PID control formula.  For the
    purpose of setting heating and cooling gains (`.get_*_gains()` and
    `.set_*_gains()`), we'll use the standard form::

      MV(t) = K_p ( e(t) + 1/T_i \int_0^t e(\tau) d\tau + T_d de(t)/dt )

    where `e(t) = SP - PV` is the error function, MV is the
    manipulated variable, SP is the setpoint, and PV is the process
    variable.

    In this formulation, the parameter units will be:

    * K_p: MV-units/PV-units (e.g. amp/K for a thermoelectric
      controller).  Don't confuse this `proportional gain` with the
      `process gain` used in `TestBackend`.
    * T_i, T_d: time (e.g. seconds)
    """
    pv_units = 'PV-units'
    mv_units = 'MV-units'

    def __init__(self):
        self._max_mv = None

    def cleanup(self):
        "Release resources and disconnect from any hardware."
        pass

    def get_pv(self):
        "Return the current process variable in PV-units"
        raise NotImplementedError()

    def get_ambient_pv(self):
        "Return the abmient (bath) status in PV-units"
        raise NotImplementedError()

    def set_max_mv(self, max):
        "Set the max manipulated variable in MV-units"
        raise NotImplementedError()

    def get_max_mvt(self):
        "Get the max manipulated variable MV-units"
        raise NotImplementedError()

    def get_mv(self):
        """Return the calculated manipulated varaible in MV-units

        The returned current is not the actual MV, but the MV that the
        controller calculates it should generate.  For example, if the
        voltage required to generate an MV current exceeds the
        controller's max voltage, then the physical current will be
        less than the value returned here.
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
    def set_mv(self, current):
        "Set the desired manipulated variable in MV-units"
        raise NotImplementedError()


class PIDMixin (object):
    def set_setpoint(self, setpoint):
        "Set the process variable setpoint in PV-units"
        raise NotImplementedError()
        
    def get_setpoint(self, setpoint):
        "Get the process variable setpoint in PV-units"
        raise NotImplementedError()

    def get_duwn_gains(self):
        """..."""
        raise NotImplementedError()

    def set_down_gains(self, proportional=None, integral=None,
                       derivative=None):
        """
        ...
        """
        raise NotImplementedError()

    def get_up_gains(self):
        """..."""
        raise NotImplementedError()

    def set_up_gains(self, proportional=None, integral=None, derivative=None):
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


class TemperatureMixin (object):
    @staticmethod
    def _convert_F_to_C(F):
        return (F - 32)/1.8

    @staticmethod
    def _convert_C_to_F(C):
        return C*1.8 + 32
