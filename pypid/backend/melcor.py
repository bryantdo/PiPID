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

import struct as _struct

import serial as _serial

from pymodbus.client.sync import ModbusSerialClient as _ModbusSerialClient

from .. import LOG as _LOG
from . import Backend as _Backend
from . import ManualMixin as _ManualMixin
from . import PIDMixin as _PIDMixin
from . import TemperatureMixin as _TemperatureMixin


class Register (object):
    def __init__(self, name, value, direction='rw', reference=None, help=None):
        self.name = name
        self.value = value
        self.direction = direction
        self.reference = reference
        self.help = help
        self.needs_decimal = False

    def __str__(self):
        return '<%s %s (%d)>' % (self.__class__.__name__, self.name, self.value)

    def encode(self, value, **kwargs):
        return value

    def decode(self, value, **kwargs):
        return value


class ChoiceRegister (Register):
    def __init__(self, *args, **kwargs):
        self.choices = kwargs.pop('choices')
        super(ChoiceRegister, self).__init__(*args, **kwargs)

    def encode(self, value, **kwargs):
        for key,v in self.choices.items():
            if v == value:
                return key
        raise ValueError(value)

    def decode(self, value, **kwargs):
        try:
            return self.choices[value]
        except KeyError:
            _LOG.error('unrecognized value %s for %s' % (value, self.name))


class FloatRegister (Register):
    def __init__(self, *args, **kwargs):
        self.decimal = kwargs.pop('decimal', None)
        self.decimal_offset = kwargs.pop('decimal_offset', None)
        super(FloatRegister, self).__init__(*args, **kwargs)
        self.needs_decimal = not self.decimal

    @staticmethod
    def _float2melcor(float, decimal=None):
        """Convert a Python float into Melcor's two's-compliment representation

        >>> m = FloatRegister._float2melcor(-3.5, decimal=10.0)
        >>> m
        65501
        >>> FloatRegister._melcor2float(m, decimal=10.0)
        -3.5
        """
        return _struct.unpack('H', _struct.pack('h', int(float * decimal)))[0]

    @staticmethod
    def _melcor2float(melcor, decimal=None):
        """Convert Melcor's two's compliment representation to a Python float

        >>> FloatRegister._melcor2float(65501, decimal=10.0)
        -3.5
        """
        return _struct.unpack('h', _struct.pack('H', melcor))[0] / decimal

    def encode(self, value, **kwargs):
        decimal = self.decimal
        if self.decimal_offset:
            decimal *= self.decimal_offset
        return self._float2melcor(value, decimal)

    def decode(self, value, decimal=None):
        decimal = self.decimal
        if self.decimal_offset:
            decimal *= self.decimal_offset
        return self._melcor2float(value, decimal)


class BoundedFloatRegister (FloatRegister):
    def __init__(self, *args, **kwargs):
        self.min = kwargs.pop('min', None)
        self.max = kwargs.pop('max', None)
        super(BoundedFloatRegister, self).__init__(*args, **kwargs)

    def encode(self, value, **kwargs):
        if value < self.min or value > self.max:
            raise ValueError('{} out of range [{}, {}] for {}'.format(
                    value, self.min, self.max, self))
        return super(BoundedFloatRegister, self).encode(value, **kwargs)

    def decode(self, value, **kwargs):
        return super(BoundedFloatRegister, self).decode(value, **kwargs)


class MelcorBackend (_Backend, _ManualMixin, _PIDMixin, _TemperatureMixin):
    """Temperature control backend for a Melcor MTCA Temperature Controller

    * PV: process temperature
    * PV-units: degrees Celsius
    * MV: controller current
    * MV-units: amps
    """
    pv_units = 'C'
    mv_units = 'A'

    # Relative register addresses from back page of Melcor Manual.
    # Then I went through Chapter 6 tables looking for missing
    # registers.  References are from Series MTCA Thermoelectric
    # Cooler Controller Instruction Manual, Revision 5.121900.
    _custom_prompt_kwargs = {
        'reference': '5.2, 6.20',
        'help': 'Setup a custom menu',
        'choices': {
            0: 'none',
            1: 'process 2',
            2: 'percent output',
            3: 'ramping set point',
            4: 'event input status',
            5: 'operation mode',
            6: 'auto-tune',
            7: 'auto-tune set point',
            8: 'set point 2',
            9: 'event set point',
            10: 'local or remote calibration mode',
            11: 'calibration offset',
            12: 'propband 1',
            13: 'integral 1',
            14: 'derivative 1',
            15: 'reset 1',
            16: 'rate 1',
            17: 'cycle time 1',
            18: 'dead band 1',
            19: 'propband 2',
            20: 'integral 2',
            21: 'derivative 2',
            22: 'reset 2',
            23: 'rate 2',
            24: 'cycle time 2',
            25: 'dead band 2',
            26: 'alarm 2 high',
            27: 'alarm 2 low',
            28: 'alarm 3 high',
            29: 'alarm 3 low',
            30: 'alarm 4 high',
            31: 'alarm 4 low',
            32: 'proportional term',
            33: 'integral term',
            34: 'derivative term',
            35: 'hysteresis 1',
            36: 'hysteresis 2',
            37: 'alarm hysteresis 2',
            38: 'alarm hysteresis 3',
            39: 'alarm hysteresis 4',
            40: 'set point 1',
            },
        }
    _registers = [
        Register('MODEL_NUMBER',                   0, direction='r', reference='6.22'),
        Register('SERIAL_NUMBER_1',                1, direction='r', reference='6.22', help='first 4 digits'),
        Register('SERIAL_NUMBER_2',                2, direction='r', reference='6.22', help='last 4 digits'),
        Register('SOFTWARE_ID_NUMBER',             3, direction='r', reference='6.22'),
        Register('SOFTWARE_REVISION',              4, direction='r', reference='6.22'),
        Register('DATE_OF_MANUFACTURE',            5, direction='r', reference='6.22', help='WEEK:YEAR (WWYY)'),
        ChoiceRegister('INPUT_2_HARDWARE_ENABLED', 9, direction='r', reference='1.2, 6.22', choices={
                0: 'none', 5: 'process event'}, help='INPUT_2 option installed'),
        ChoiceRegister('OUTPUT_1_HARDWARE',       16, direction='r', reference='6.23', choices={
                0: 'none', 1: 'relay', 2: 'solid state', 3: 'dc', 4: 'process'}),
        ChoiceRegister('OUTPUT_2_HARDWARE',       17, direction='r', reference='6.23', choices={
                0: 'none', 1: 'relay', 2: 'solid state', 3: 'dc', 4: 'process'}),
        ChoiceRegister('OUTPUT_3_HARDWARE',       18, direction='r', reference='6.23', choices={
                0: 'none', 1: 'relay'}),
        ChoiceRegister('OUTPUT_4_HARDWARE',       19, direction='r', reference='5.9, 6.23', choices={
                0: 'none', 1: 'relay', 4: 'process', 6: '485', 7: '232'},
                       help='Retransmit option installed'),
        Register('DISABLE_NONVOLATILE_MEM',       24, reference=''),
        FloatRegister('PROCESS_1',               100, direction='r', reference='6.3', help='Current temp (input to INPUT_1) (mdbl)'),
        Register('ERROR_1',                      101, reference=''),
        Register('PERCENT_OUTPUT',               103, direction='r', reference='5.4, 6.4', help="% of controller's rated maximum power/current"),
        Register('ACTUAL_2',                     104, reference=''),
        FloatRegister('PROCESS_2',               105, direction='r', reference='6.4', help='Value of signal input to INPUT_2'),
        Register('ALARM_2_STATUS',               106, reference=''),
        Register('ALARM_3_STATUS',               110, reference=''),
        Register('ALARM_4_STATUS',               114, reference=''),
        Register('OPERATION_MODE',               200, reference='?'),
        ChoiceRegister('EVENT_INPUT_STATUS',     201, direction='r', reference='6.4', choices={
                1:True, 0:False}, help='Whether EVENT_FUNCTION satisfies EVENT_CONDITION'),
        FloatRegister('REMOTE_SET_POINT',        202, direction='r', reference='6.3', help='Or event set point'),
        Register('RAMPING_SET_POINT',            203, direction='r', reference='6.4', help='Active if RAMPING_MODE not set to OFF'),
        # NOTE: sometimes the *_TERM_1 registers blib to 10x the predicted value.  I don't know why yet...
        FloatRegister('PID_POWER_1',             204, reference='Not in manual', help='Calculated output current %, active when Factory->Diagnostic->Troubleshooting == 1, but no modbus register for Troubleshooting (6.24).', decimal=10.),
        FloatRegister('PROP_TERM_1',             205, reference='Not in manual', help='(Tset-Tcur)/Tprop see temperature.tempControl.getFeedbackTerms(), active when Troubleshooting == 1.', decimal=1.),
        FloatRegister('INTEGRAL_TERM_1',         206, reference='', decimal=1.),
        FloatRegister('DERIVATIVE_TERM_1',       207, reference='', decimal=1.),
        Register('SYSTEM_ERROR',                 209, reference=''),
        Register('OPEN_LOOP_ERROR',              210, reference=''),
        FloatRegister('SET_POINT_1',             300, reference='5.7 6.3', help='Set-point for INPUT_1'),
        ChoiceRegister('AUTO_MANUAL_OP_MODE',    301, direction='r', reference='6.4', help='Select control mode', choices={0: 'PID', 1: 'manual'}),
        Register('AUTO_TUNE_SETPOINT',           304, reference='6.5', help='Set auto tune setpoint as % of current set point (default 90%)'),
        ChoiceRegister('AUTO_TUNE_START_1',      305, reference='6.5', help='Initiate or cancel auto-tune.  Active if AUTO_MANUAL_OP_MODE is Auto (PID)', choices = {0: 'off or cancel', 1: 'initiate', 2: 'set only PID 1', 3: 'set only PID2'}),
        FloatRegister('EVENT_SET_POINT_1',       306, reference='6.2', decimal=1.),
        FloatRegister('BOOST_SET_POINT_1',       309, reference='1.2', help='Optional, on back plate'),
        Register('MANUAL_SET_POINT',             310, reference='6.3', help='If AUTO_MANUAL_OP_MODE is MANUAL (manual)'),
        Register('CLEAR_INPUT_ERRORS',           311, reference=''),
        ChoiceRegister('LOCAL_REMOTE_1',         316, reference='5.9, 6.5', choices={
                    0: 'local', 1: 'remote'}, help='Selects active setpoint.  Active if INPUT_2 is not OFF or EVENT'),
        FloatRegister('SET_POINT_2',             319, reference='6.5', help='?boost setpoint? Active if both output 1 and output 2 are set to HEAT, or both are set to COOL, or if INPUT_2 is set to EVENT and EVENT_FUNCTION to SP'),
        FloatRegister('ALARM_2_LOW',             321, reference='5.18, 6.2, 6.8'),
        FloatRegister('ALARM_2_HIGH',            322, reference='5.18, 6.2, 6.8'),
        Register('CLEAR_ALARMS',                 331, reference=''),
        Register('SILENCE_ALARMS',               332, reference=''),
        FloatRegister('ALARM_3_LOW',             340, reference='5.18, 6.2, 6.9'),
        FloatRegister('ALARM_3_HIGH',            341, reference='5.18, 6.2, 6.9'),
        BoundedFloatRegister('PROPBAND_1',       500, reference='6.2, 6.5', help='Width of proportional band in PID control(mdbl)', min=0, max=9999),
        BoundedFloatRegister('INTEGRAL_1',       501, reference='6.6', help='Set integral time in minutes for output 1', decimal=100., min=0, max=99.99),
        BoundedFloatRegister('RESET_1',          502, reference='6.6', help='Set reset time in repeats per minute for output 1 if UNITS_TYPE set to US', decimal=100., min=0, max=99.99),
        BoundedFloatRegister('DERIVATIVE_1',     503, reference='6.6', help='Set derivative time in minutes', decimal=100., min=0, max=9.99),
        BoundedFloatRegister('RATE_1',           504, reference='6.6', decimal=100., min=0, max=9.99),
        BoundedFloatRegister('DEAD_BAND_1',      505, reference='6.2, 6.7', min=0, max=9999),
        FloatRegister('CYCLE_TIME_1',            506, reference='6.6', help='Valid range depends on output type.  Relay: 5.0 to 60.0, solid state: 0.1 to 60.0.  Not worth the extra call to automate this check.', decimal=10.),
        BoundedFloatRegister('HYSTERESIS_1',     507, reference='6.2, 6.6', min=1, max=9999),
        ChoiceRegister('BURST_1',                509, reference='5.16, 6.6', choices={
                0: 'no', 1: 'yes'}),
        BoundedFloatRegister('PROPBAND_2',       510, reference='6.2, 6.7', min=0, max=9999),
        BoundedFloatRegister('INTEGRAL_2',       511, reference='6.7', decimal=100., min=0, max=99.99),
        BoundedFloatRegister('RESET_2',          512, reference='6.7', decimal=100., min=0, max=99.99),
        BoundedFloatRegister('DERIVATIVE_2',     513, reference='6.7', decimal=100., min=0, max=9.99),
        BoundedFloatRegister('RATE_2',           514, reference='6.7', decimal=100., min=0, max=9.99),
        BoundedFloatRegister('DEAD_BAND_2',      515, reference='6.2, 6.8', min=0, max=9999),
        FloatRegister('CYCLE_TIME_2',     516, reference='6.8',  help='Valid range depends on output type.  Relay: 5.0 to 60.0, solid state: 0.1 to 60.0.  Not worth the extra call to automate this check.', decimal=10.),
        BoundedFloatRegister('HYSTERESIS_2',     517, reference='6.2, 6.8', min=1, max=9999),
        ChoiceRegister('BURST_2',                519, reference='5.16, 6.7', choices={
                0: 'no', 1: 'yes'}),
        Register('SENSOR_TYPE_1',                600, reference='5.7', help='Sensor used for INPUT_1'),
        Register('INPUT_1',                      601, reference='5.7', help='Temperature measurement'),
        FloatRegister('RANGE_LOW_1',             602, reference='5.7, 6.2, 6.11', help='Minimum SET_POINT_1'),
        FloatRegister('RANGE_HIGH_1',            603, reference='5.7, 6.2, 6.11', help='Maximum SET_POINT_1'),
        BoundedFloatRegister('INPUT_SOFTWARE_FILTER_1', 604, reference='5.6, 6.2, 6.11, ', help='Averaging to smooth INPUT_1 (positive only affect monitor values, negative affect both monitor and control)', decimal=10., min=-60, max=60),
        FloatRegister('CALIBRATION_OFFSET_1',    605, reference='5.5, 6.2, 6.5', help='Offset added to INPUT_1'),
        ChoiceRegister('DECIMAL_1',              606, reference='6.11', choices={
                0: 1., 1: 10., 2: 1., 3: 10., 4: 100., 5: 1000.}),
        ChoiceRegister('INPUT_ERROR_LATCHING',   607, reference='6.18', choices={
                    0: 'latching', 1: 'no latching'}),
        ChoiceRegister('INPUT_2',                611, reference='5.8, 6.11', choices={
                0: 'off', 1: 'event', 2: '4-20mA', 3: '0-20mA', 4: '0-5V dc', 5: '1-5V dc', 6: '0-10V dc'},
                       help='For external control'),
        FloatRegister('RANGE_LOW_2',             612, reference='5.9, 6.2, 6.12', help='Minimum INPUT_2 signal'),
        FloatRegister('RANGE_HIGH_2',            613, reference='5.9, 6.2, 6.12', help='Maximum INPUT_2 signal'),
        FloatRegister('CALIBRATION_OFFSET_2',    615, reference='5.5,, 6.2, 6.12', help='Offset added to INPUT_2'),
        ChoiceRegister('OUTPUT_1',               700, reference='6.13', choices={
                    0: 'heat', 1: 'cool'}),
        ChoiceRegister('PROCESS_1_TYPE',         701, reference='6.13', choices={
                0: '4-20mA', 1: '0-20mA', 2: '0-5V dc', 3: '1-5V dc', 4: '0-10V dc'}),
        Register('HIGH_LIMIT_SET_POINT',         702, reference=''),
        FloatRegister('POWER_LIMIT_SET_POINT',   713, reference='5.4, 6.2, 6.19', help='Temperature set point for power limits'),
        FloatRegister('HIGH_POWER_LIMIT_ABOVE',  714, reference='5.4', help='% limit when above PLSP'),
        FloatRegister('HIGH_POWER_LIMIT_BELOW',  715, reference='5.4', help='% limit when below PLSP'),
        ChoiceRegister('OUTPUT_2',               717, reference='6.13', choices={
                0: 'off', 1: 'heat', 2: 'cool', 3: 'alarm'}),
        ChoiceRegister('PROCESS_2_TYPE',         718, reference='6.13', choices={
                0: '4-20mA', 1: '0-20mA', 2: '0-5V dc', 3: '1-5V dc', 4: '0-10V dc'},
                 help='The manual claims: (0: 4-20mA, 1: 0-20mA, 2: 0-10V dc, 3: 0-5V dc, 4: 1-5V dc), but I think it has the same sttings as PROCESS_1_TYPE, because that matches the results I expect when setting PROCESS_2_TYPE from software while watching the relevant display menu'),
        ChoiceRegister('ALARM_2_TYPE',           719, reference='5.19, 6.13', choices={
                0: 'process', 1: 'deviation'}, help='Select alarm type.  A process alarm responds when the temperature leaves a fixed range.  A deviation alarm responds when the temperature deviates from the set point by a set number of degrees'),
        FloatRegister('ALARM_HYSTERESIS_2',      720, reference='5.18, 6.2, 6.13', help='Set the switching histeresis for the alarm output.  This defines a band on the inside of the alarm set point.  When the process temperature is in this band, the alarm state will not change.'),
        ChoiceRegister('LATCHING_2',             721, reference='5.19, 6.14', choices={
                0: 'no', 1: 'yes'}),
        ChoiceRegister('SILENCING_2',            722, reference='5.20, 6.14', choices={
                0: 'no', 1: 'yes'}),
        ChoiceRegister('ALARM_ACTIVE_SIDES_2' ,  723, reference='6.14', choices={
                0: 'both', 1: 'high', 2: 'low'},
                 help='Select which side or sides the alarm setpoints can be programmed for'),
        ChoiceRegister('ALARM_LOGIC_2',          724, reference='6.14', choices={
                0: 'de-energize', 1: 'energize'},
                 help='Select alarm 2 output condition in the alarm state.  De-energizing is the failsafe behaviour.'),
        ChoiceRegister('ALARM_ANNUNCIATION_2',   725, reference='6.14', choices={
                0: 'no', 1: 'yes'}),
        ChoiceRegister('OUTPUT_3',               734, reference='6.15', choices={
                0: 'off', 1: 'alarm'}),
        ChoiceRegister('ALARM_3_TYPE',           736, reference='5.19, 6.15', choices={
                0: 'process', 1: 'deviation'}, help='Select alarm type.  A process alarm responds when the temperature leaves a fixed range.  A deviation alarm responds when the temperature deviates from the set point by a set number of degrees'),
        FloatRegister('ALARM_HYSTERESIS_3',      737, reference='5.18, 6.2, 6.15', help='Set the switching histeresis for the alarm output.  This defines a band on the inside of the alarm set point.  When the process temperature is in this band, the alarm state will not change.'),
        ChoiceRegister('LATCHING_3',             738, reference='5.19, 6.15', choices={
                0: 'no', 1: 'yes'}),
        ChoiceRegister('SILENCING_3',            739, reference='5.20, 6.15', choices={
                0: 'no', 1: 'yes'}),
        ChoiceRegister('ALARM_ACTIVE_SIDES_3',   740, reference='6.15', choices={
                0: 'both', 1: 'high', 3: 'low'},
                 help='Select which side or sides the alarm setpoints can be programmed for'),
        ChoiceRegister('ALARM_LOGIC_3',          741, reference='6.16', choices={
                0: 'de-energize', 1: 'energize'},
                 help='Select alarm 3 output condition in the alarm state.  De-energizing is the failsafe behaviour.'),
        ChoiceRegister('ALARM_ANNUNCIATION_2',   742, reference='6.16', choices={
                0: 'no', 1: 'yes'}),
        ChoiceRegister('UNITS_TYPE',             900, reference='6.18', choices={
                1: 'US, use reset and rate', 2: 'SI, use integral and derivative'}),
        ChoiceRegister('C_OR_F',                 901, reference='6.18', choices={
                0: 'fahrenheit', 1: 'celsius'}),
        ChoiceRegister('FAILURE_MODE',           902, reference='?.?, 6.18', choices={
                0: 'bumpless', 1: 'manual', 2: 'off'}),
        Register('MANUAL_DEFAULT_POWER',         903, reference='6.19'),
        ChoiceRegister('OPEN_LOOP_DETECT',       904, reference='5.21, 6.19', choices={
                0: 'on', 1: 'off'}),
        ChoiceRegister('EVENT_FUNCTION',        1060, reference='5.8, 6.12', choices={
                0: 'none',
                1: 'switch to event set point',
                2: 'turn off control outputs and disable alarms',
                3: 'turn off control outputs',
                4: 'lock keyboard',
                5: 'switch to manual mode',
                6: 'initiate an auto-tune',
                7: 'clear alarm',
                8: 'lock everything except primary set point',
                },
                       help='Selects response to INPUT_2'),
        ChoiceRegister('EVENT_CONDITION',       1061, direction='r', reference='5.8, 6.12', choices={
                0: 'low', 1: 'high', 2: 'rise', 3: 'fall'},
                       help='What behavior triggers Events'),
        ChoiceRegister('RAMPING_MODE',          1100, reference='6.19', choices={
                0: 'off', 1: 'startup only', 2: 'startup or setpoint change'}),
        Register('RAMP_RATE',                   1101, reference=''),
        ChoiceRegister('RAMP_SCALE',            1102, reference='6.19', choices={
                0: 'minute', 1: 'hour'}),
        Register('SET_POINT_MENU_LOCK',         1300, reference='6.21'),
        Register('OPERATIONS_PAGE_MENU_LOCK',   1301, reference=''),
        Register('SETUP_PAGE_LOCK',             1302, reference=''),
        Register('CUSTOM_MENU_LOCK',            1304, reference=''),
        Register('CALIBRATION_MENU_LOCK',       1305, reference=''),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_1',  1400, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_2',  1401, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_3',  1402, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_4',  1403, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_5',  1404, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_6',  1405, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_7',  1406, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_8',  1407, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_9',  1408, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_10', 1409, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_11', 1410, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_12', 1411, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_13', 1412, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_14', 1413, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_15', 1414, **_custom_prompt_kwargs),
        ChoiceRegister('CUSTOM_PROMPT_NUMBER_16', 1415, **_custom_prompt_kwargs),
        FloatRegister('AMBIENT_TEMPERATURE',    1500, direction='r', reference='6.23', help='Always in deg F, regardless of C_OR_F', decimal=10.),
        Register('AMBIENT_A_D_COUNTS',          1501, direction='r', reference='6.23'),
        Register('CHANNEL_1_A_D_COUNTS',        1504, direction='r', reference='6.24'),
        Register('CHANNEL_2_A_D_COUNTS',        1505, direction='r', reference='6.24'),
        ChoiceRegister('TEST_DISPLAY',          1513, reference='6.23', choices={
                0: 'off', 1: 'on'}, help='Cyclic display test'),
        ChoiceRegister('TEST_OUTPUT',           1514, reference='6.23', choices={
                0: 'none', 1: 'output 1', 2: 'outptut 2', 3: 'output 3',
                4: 'output 4', 5: 'all outputs'},
                       help='Turns onn specific output'),
        Register('LINE_FREQUENCY',              1515, direction='r', reference='6.24', help='AC line freq in Hz'),
        ChoiceRegister('RESTORE_FACTORY_CALIBRATION', 1601, direction='w', reference='6.24', choices={
                0: 'no', 1: 'yes'}),
        Register('DEFAULT_SETTINGS',            1602, direction='w', reference='6.24'),
        ChoiceRegister('OVERLOADED_CALIBRATION_1', 1603, direction='w', reference='6.24, 6.25', choices={
                0: 'no',
                1: 'thermocouple, 0mV',
                2: 'thermocouple, 50mV',
                3: 'thermocouple, 32deg',
                4: 'ground',
                5: 'lead resistance',
                6: 'RTD, 15 Ohms',  # RTD = Resistance Temp. Detector
                7: 'RTD, 380 Ohms',
                8: 'process 1, 0V',
                9: 'process 1, 10V',
                10: 'process 1, 4mA',
                11: 'process 1, 20mA',
                }),
        Register('OUTPUT_CALIBRATION_1_4MA',    1604, direction='w', reference='6.26'),
        Register('OUTPUT_CALIBRATION_1_20MA',   1605, direction='w', reference='6.26'),
        Register('OUTPUT_CALIBRATION_1_1V',     1606, direction='w', reference='6.26'),
        Register('OUTPUT_CALIBRATION_1_10V',    1607, direction='w', reference='6.27'),
        ChoiceRegister('OVERLOADED_CALIBRATION_2', 1608, direction='w', reference='6.26', choices={
                0: 'no',
                1: 'process 2, 0V',
                2: 'process 2, 10V',
                3: 'process 2, 4mA',
                4: 'process 2, 20mA',
                }),
        Register('OUTPUT_CALIBRATION_2_4MA',    1609, direction='w', reference='6.27'),
        Register('OUTPUT_CALIBRATION_2_20MA',   1610, direction='w', reference='6.27'),
        Register('OUTPUT_CALIBRATION_2_1V',     1611, direction='w', reference='6.27'),
        Register('OUTPUT_CALIBRATION_2_10V',    1612, direction='w', reference='6.27'),
        Register('OUTPUT_CALIBRATION_4_4MA',    1619, direction='w', reference='6.27'),
        Register('OUTPUT_CALIBRATION_4_20MA',   1620, direction='w', reference='6.27'),
        Register('OUTPUT_CALIBRATION_4_1V',     1621, direction='w', reference='6.27'),
        Register('OUTPUT_CALIBRATION_4_10V',    1622, direction='w', reference='6.27'),
        FloatRegister('HIGH_RESOLUTION',        1707, direction='r', reference='6.23', help='High resolution input value', decimal_offset=10.),
        ]
    del(_custom_prompt_kwargs)
    _register = dict((r.name, r) for r in _registers)

    def __init__(self, controller=1, device='/dev/ttyS0', baudrate=9600):
        """
        controller : MTCA controller ID
        device     : serial port you're using to connect to the controller
        baudrate   : baud rate for which you've configured your controller
        """
        # the rated max current from controller specs
        self._spec_max_current = 4.0  # Amps

        self._controller = controller

        # from the Melcor Manual, A.4 (p96), messages should be coded
        # in eight-bit bytes, with no parity bit, and one stop bit
        # (8N1).
        self._client = _ModbusSerialClient(
            method='rtu',
            port=device,  # '/dev/ttyS0' or 0
            bytesize=_serial.EIGHTBITS,
            parity=_serial.PARITY_NONE,
            stopbits=_serial.STOPBITS_ONE,
            baudrate=baudrate,
            timeout=0.5,
            )

        self._decimal = None

    def _read(self, register_name):
        register = self._register[register_name]
        if 'r' not in register.direction:
            raise ValueError(register_name)
        if register.needs_decimal:
            if not self._decimal:
                self._decimal = self._get_decimal()
            register.decimal = self._decimal
        rc = self._client.read_holding_registers(
            address=register.value, count=1, unit=self._controller)
        assert rc.function_code < 0x80
        value = rc.registers[0]
        v = register.decode(value, decimal=self._decimal)
        _LOG.info('read %s: %s %s (%s)' % (register_name, rc, v, rc.registers))
        return v

    def _write(self, register_name, value):
        register = self._register[register_name]
        if 'w' not in register.direction:
            raise ValueError(register_name)
        if register.needs_decimal and not self._decimal:
            self._decimal = self._get_decimal()
        v = register.encode(value, decimal=self._decimal)
        _LOG.info('write %s: %s (%s)' % (register_name, v, value))
        rc = self._client.write_register(
            address=register.value, value=v, unit=self._controller)
        assert rc.function_code < 0x80

    def _get_decimal(self):
        return self._read('DECIMAL_1')

    # Support for Backend methods

    def get_pv(self):
        return self._read('HIGH_RESOLUTION')

    def get_ambient_pv(self):
        return self._convert_F_to_C(self._read('AMBIENT_TEMPERATURE'))

    def set_max_mv(self, max):
        """Set the max current in Amps

        0.2 A is the default max current since it seems ok to use
        without fluid cooled heatsink.  If you are cooling the
        heatsink, use 1.0 A, which seems safely below the peltier's
        1.2 A limit.

        Note to Melcor enthusiasts: this method set's both the 'above'
        and 'below' limits.
        """
        max_percent = max / self._spec_max_current * 100
        self._write('HIGH_POWER_LIMIT_ABOVE', max_percent)
        self._write('HIGH_POWER_LIMIT_BELOW', max_percent)
        self._max_current = max

    def get_max_mv(self):
        percent = self._read('HIGH_POWER_LIMIT_ABOVE')
        above = percent/100. * self._spec_max_current
        percent = self._read('HIGH_POWER_LIMIT_BELOW')
        below = percent/100. * self._spec_max_current
        #setpoint = self._read('POWER_LIMIT_SET_POINT')
        assert above == below, 'Backend() only expects a single power limit'
        self._max_current = above
        return above

    def get_mv(self):
        pout = self._read('PERCENT_OUTPUT')
        cur = self._spec_max_current * pout / 100.0
        return cur

    def get_modes(self):
        register = self._register['AUTO_MANUAL_OP_MODE']
        return sorted(register.choices.values())

    def get_mode(self):
        return self._read('AUTO_MANUAL_OP_MODE')

    def set_mode(self, mode):
        self._write('AUTO_MANUAL_OP_MODE', mode)

    def dump_configuration(self):
        for register in self._registers:
            if 'r' in register.direction:
                value = self._read(register.name)
                print('%s\t%s' % (register.name, value))

    # ManualMixin methods

    def set_mv(self, current):
        if current > self._spec_max_current:
            raise ValueError('current {} exceeds spec maximum {}'.format(
                    current, self._spec_max_current))
        pout = current / self._spec_max_current * 100.0
        self._write('REG_MANUAL_SET_POINT', pout)

    # PIDMixin methods

    def set_setpoint(self, setpoint):
        self._write('SET_POINT_1', setpoint)

    def get_setpoint(self):
        return self._read('SET_POINT_1')

    def _set_gains(self, output, proportional=None, integral=None,
                   derivative=None):
        """
        (output, proportional, integral, derivative, dead_band) -> None
        output       : 1 (cooling) or 2 (heating)
        proportional : propotional gain band in amps per degrees C
        integral     : integral weight in minutes (0.00 to 99.99)
        derivative   : derivative weight in minutes (? to ?)

        Don't use derivative, dead time.
        Cycle time?
        Histerysis?
        Burst?

        See 5.10 and the pages afterwards in the manual for Melcor's
        explanation.  The integral with respect to t' is actually only
        from the time that T_samp has been with T_prop of T_set (not
        -inf), and
        """
        if proportional is not None:
            max_current = self.get_max_current()
            propband = max_current/proportional
            propband_name = 'PROPBAND_%d' % output
            register = self._register[propband_name]
            if propband > register.max:
                # round down, to support bang-bang experiments
                _LOG.warn(
                    'limiting propband %d to maximum: {:n} -> {:n} C'.format(
                        propband, register.max))
                propband = register.max
            self._write(propband_name, propband)
        if integral is not None:
            self._write('INTEGRAL_%d' % output, integral)
        if derivative is not None:
            self._write('DERIVATIVE_%d' % output, derivative)

    def _get_gains(self, output):
        propband = self._read('PROPBAND_%d' % output)
        integral = self._read('INTEGRAL_%d' % output)
        derivative = self._read('DERIVATIVE_%d' % output)
        max_current = self.get_max_current()
        proportional = max_current/propband
        return (proportional, integral, derivative)

    def set_down_gains(self, proportional=None, integral=None,
                       derivative=None):
        self._set_gains(
            output=1, proportional=proportional, integral=integral,
            derivative=derivative)

    def get_down_gains(self):
        return self._get_gains(output=1)

    def set_up_gains(self, proportional=None, integral=None, derivative=None):
        self._set_gains(
            output=2, proportional=proportional, integral=integral,
            derivative=derivative)

    def get_up_gains(self):
        return self._get_gains(output=2)

    def get_feedback_terms(self):
        """
        """
        pid = int(self._read('PID_POWER_1'))
        prop = int(self._read('PROP_TERM_1'))
        ntgrl = int(self._read('INTEGRAL_TERM_1'))
        deriv = int(self._read('DERIVATIVE_TERM_1'))
        return (pid, prop, ntgrl, deriv)

    def clear_integral_term(self):
        # The controller resets the integral term when the temperature
        # is outside the propbands
        _LOG.debug('clearing integral term')
        cp,ci,cd = self.get_cooling_gains()
        hp,hi,hd = self.get_heating_gains()
        sp = self.get_setpoint()
        small_temp_range = 0.1
        max_current = self.get_max_current()
        p = max_current / small_temp_range
        self.set_cooling_gains(proportional=p)
        self.set_heating_gains(proportional=p)
        while True:
            _LOG.debug('waiting for an out-of-propband temperature')
            if abs(self.get_temp() - sp) > small_temp_range:
                break  # we're out of the propband, I-term resets
        self.set_cooling_gains(proportional=cp)
        self.set_heating_gains(proportional=hp)
        _LOG.debug('integral term cleared')

    # utility methods

    def sanity_check(self):
        "Check that some key registers have the values we expect"
        self._sanity_check('UNITS_TYPE',   'SI, use integral and derivative')
        self._sanity_check('C_OR_F',       'celsius')
        self._sanity_check('FAILURE_MODE', 'off')
        self._sanity_check('RAMPING_MODE', 'off')
        self._sanity_check('OUTPUT_1',     'cool')
        self._sanity_check('OUTPUT_2',     'heat')
        self._sanity_check('AUTO_MANUAL_OP_MODE',  'PID')

    def _sanity_check(self, register_name, expected_value):
        value = self._read(register_name)
        if value != expected_value :
            _LOG.error('invalid value %s for %s (expected %s)'
                       % (value, register_name, expected_value))
            raise ValueError(value)
