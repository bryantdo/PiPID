# Copyright (C) 2011 W. Trevor King <wking@drexel.edu>
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

"Assorted PID tuning rules"


def ziegler_nichols_step_response(process_gain, dead_time, decay_time, mode='PID'):
    K,L,T = (process_gain, dead_time, decay_time)
    r = L / T
    if r > 1:
        _LOG.warn(('Ziegler-Nichols not a good idea when '
                   'dead-time/decay_time = {:n}').format(r))
    pkern = 1/(K*r)
    if mode == 'P':
        return (pkern, float('inf'), 0)
    elif mode == 'PI':
        return (0.9*pkern, 3.3*L, 0)
    elif mode == 'PID':
        return (1.2*pkern, 2*L, L/2.)
    raise ValueError(mode)

def ziegler_nichols_bang_bang_response(amplitude, period, max_current,
                                       mode='PID'):
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

def ziegler_nichols_ultimate_cycle_response(proportional, period):
    """
    proportional : float
        critical P-only process_gain (ultimate process_gain, for sustained oscillation)
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

def cohen_coon_step_response(process_gain, dead_time, decay_time, mode='PID'):
    K,L,T = (process_gain, dead_time, decay_time)
    r = L/T
    pkern = 1/(K*r)
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

def wang_juang_chan_step_response(process_gain, dead_time, decay_time, mode='PID'):
    """Wang-Juang-Chan tuning
    """
    K,L,T = (process_gain, dead_time, decay_time)
    if mode == 'PID':
        return ((0.7303+0.5307*T/L)*(T+0.5*L)/(K*(T+L)),
                T + 0.5*L,
                0.5*L*T / (T + 0.5*L))
    raise ValueError(mode)        

