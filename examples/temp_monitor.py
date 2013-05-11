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

"""Log control and ambient temperature every 10 seconds.

usage: python temp_monitor.py
"""

import time

from pypid.backend import get_backend


b = get_backend('melcor')()
period = 10

with open('temp_monitor.log', 'a') as f:
    last = time.time()
    last -= last % period
    next_time = last + period
    while True:
        time.sleep(next_time - time.time())
        tstr = time.strftime('%Y-%m-%d %H:%M:%S')
        temp = str(b.get_pv())
        ambient = str(b.get_ambient_pv())
        f.write('\t'.join([tstr, temp, ambient]) + '\n')
        f.flush()
        print('\t'.join([tstr, temp, ambient]))
        next_time += period
