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

import logging as _logging


__version__ = '0.3'


LOG = _logging.getLogger('tempcontrol')
"Temperature-control logger"

#LOG.setLevel(_logging.WARN)
LOG.setLevel(_logging.DEBUG)
_formatter = _logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s')

_stream_handler = _logging.StreamHandler()
_stream_handler.setLevel(_logging.DEBUG)
_stream_handler.setFormatter(_formatter)
LOG.addHandler(_stream_handler)

_syslog_handler = None


def _set_handler(name='stream'):
    if name == 'syslog':
        if not _syslog_handler:
            _syslog_handler = _logging_handlers.SysLogHandler()
            _syslog_handler.setLevel(_logging.DEBUG)
        LOG.handlers = [_syslog_handler]
    elif name == 'stream':
        LOG.handlers = [_stream_handler]
    else:
        raise ValueError(name)
    LOG.info('setup logging handler: %s' % name)
