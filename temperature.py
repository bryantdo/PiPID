import Melcor
import time
import data_logger

log_dir = "/home/wking/rsrch/data/temperature"

class error (Exception) :
    "Errors with the temperature controller"
    pass

class errorMmelcor (error) :
    pass
class errorOutOfRange (error) :
    pass

def _check1(functionCall) :
    (err, val) = functionCall
    if err != 0 :
    	raise errorMelcor
    return val

def _check0(functionCall) :
    err = functionCall
    if err != 0 :
    	raise errorMelcor


def melcor2double(value) :
    (err, doub) = Melcor.melcor2double(value)
    if err != 0 :
    	raise errorMelcor, "Error converting melcor to double"
    return doub
def double2melcor(doub) :
    (err, val) = Melcor.double2melcor(doub)
    if err != 0 :
    	raise errorMelcor, "Error converting double to melcor"
    return val

def check_range(raw_output, min, max) :
    if raw_output < min :
        raise errorOutOfRange, '%g < %g' % (raw_output, min)
    if raw_output > max :
        raise errorOutOfRange, '%g > %g' % (raw_output, max)

class tempController :
    "Pretty wrappers for controlling a Melcor MTCA Temperature Controller"
    def __init__(self, controller=1, device='/dev/ttyS0', maxCurrent=0.2) :
        """
        (controller, device, maxCurrent) -> (tempController instance)
        controller : MTCA controller Id
        device     : serial port you're using to connect to the controller
        maxCurrent : initial maximum allowed current (in Amps)
        Set maxCurrent = None if you don't want to adjust from it's prev. value.

    	0.2 A is the default max current since it seems ok to use without fluid
        cooled heatsink.  If you are cooling the heatsink, use 1.0 A, which seems
        safely below the peltier's 1.2 A limit.
        """
        self.verbose = False
    	self.setpoint = 20.0 # degrees C
    	self.Tmin = 5.0 # setup some protective bounds for sanity checks
    	self.Tmax = 50.0
    	self.specMaxCur = 4.0 # Amps, the rated max current from controller specs
    	self.T = Melcor.tempController(controller, device)
        if maxCurrent != None : # if None, just leave maxCurrent at it's prev. val.
            self.setMaxCurrent(maxCurrent) # Amps
    def getTemp(self) :
        "Returns the current process temperature in degrees Celsius"
    	val = self.read(Melcor.REG_HIGH_RESOLUTION)
    	temp = val/100.0
    	return temp
    def getAmbientTemp(self) :
        "Returns room temperature in degrees Celsius"
    	val = self.read(Melcor.REG_AMBIENT_TEMPERATURE)
    	# convert (Fahrenheit*10) to Celsius
    	return (val/10.0 - 32)/1.8
    def setSetpoint(self, setpoint) :
        "Set the temperature setpoint in degrees Celsius"
    	val = double2melcor(setpoint)
    	self.write(Melcor.REG_SET_POINT_1, val)
    def getSetpoint(self) :
        "Get the temperature setpoint in degrees Celsius"
    	val = self.read(Melcor.REG_SET_POINT_1)
    	return melcor2double(val)
    def setMaxCurrent(self, maxCur) :
        """
        Set the max current in Amps.
        (Note to Melcor enthusiasts: set's both the 'above' and 'below' limits)
        """
    	maxPercent = maxCur / self.specMaxCur * 100
    	val = double2melcor(maxPercent)
    	self.write(Melcor.REG_HIGH_POWER_LIMIT_ABOVE, val)
    	self.write(Melcor.REG_HIGH_POWER_LIMIT_BELOW, val)
    	self.maxCurrent = maxCur
    def getMaxCurrent(self) :
        """
        () -> (currentLimitAbove (A), currentLimitBelow (A), currentLimitSetpoint (deg C))
        """
    	per = self.read(Melcor.REG_HIGH_POWER_LIMIT_ABOVE)
        curLimAbove = melcor2double(per)/100.0 * self.specMaxCur
    	per = self.read(Melcor.REG_HIGH_POWER_LIMIT_BELOW)
        curLimBelow = melcor2double(per)/100.0 * self.specMaxCur
        val = self.read(Melcor.REG_POWER_LIMIT_SETPOINT)
        curLimSet = melcor2double(val)
        return (curLimAbove, curLimBelow, curLimSet)
    def getPercentCurrent(self) :
        """
        Returns the percent of rated max current being output.
        See getCurrent()
        """
    	val = self.read(Melcor.REG_PERCENT_OUTPUT)
        return val
    def getCurrent(self) :
        """
        The returned current is not the actual current,
        but the current that the temperature controller
        calculates it should generate.
        If the voltage required to generate that current
        exceeds the controllers max voltage (15V on mine),
        then the physical current will be less than the
        value returned here.
        """
    	percentOutput = self.getPercentCurrent()
    	return self.specMaxCur * percentOutput / 100
    def setCoolingGains(self, propband=0.1, integral=0, derivative=0) :
        """
        (propband, integral, derivative, dead_band) -> None
        propband   : propotional gain band in degrees C
        integral   : integral weight in minutes (0.00 to 99.99)
        derivative : derivative weight in minutes (? to ?)
        See 5.10 and the pages afterwards in the manual for Melcor's explaination.
        Formula (from Cornell BioPhys El Producto Beamline notes)
        P_cout = -1/T_prop * [ (T_samp - T_set)
                               + 1/t_int * int_-inf^t (T_samp(t')-T_set(t')) dt'
                               + t_deriv * dT_samp/dt
        Where P_cout is the percent of the rated max current that the controller
         would like to output if you weren't limiting it,
        T_prop is the propband input to this function, 
        T_samp is the measured temperature of the sample in deg C, 
        T_set is the setpoint in deg C,
        t_int is the integral input to this function,
        the integral with respect to t' is actually only from the time that
         T_samp has been with T_prop of T_set (not -inf), and
        t_deriv is the derivative input to this function.

        Cooling is output 1
        """
        check_range(propband, 0, 99.9)
        check_range(integral, 0, 99.99)
        check_range(derivative, 0, 99.99)

    	val = double2melcor(propband)
    	self.write(Melcor.REG_PROPBAND_1, val)
        val = int(integral * 100)
    	self.write(Melcor.REG_INTEGRAL_1, val)
        val = int(derivative * 100)
    	self.write(Melcor.REG_DERIVATIVE_1, val)
    def getCoolingGains(self) :
        "() -> (propband, integral, derivative)"
    	val = self.read(Melcor.REG_PROPBAND_1)
    	propband = melcor2double(val)
    	val = self.read(Melcor.REG_INTEGRAL_1)
        integral = val/100.0
    	val = self.read(Melcor.REG_DERIVATIVE_1)
        derivative = val/100.0
        return (propband, integral, derivative)
    def setHeatingGains(self, propband=0.1, integral=0, derivative=0) :
        """
        (propband, integral, derivative, dead_band) -> None
        propband   : propotional gain band in degrees C
        integral   : integral weight in minutes (0.00 to 99.99)
        derivative : derivative weight in minutes (? to ?)
        Don't use derivative, dead time.
        Cycle time?
        Histerysis?
        Burst?
        See 5.10 and the pages afterwards in the manual for Melcor's explaination.
        Formula (from Cornell BioPhys El Producto Beamline notes)
        P_cout = -1/T_prop * [ (T_samp - T_set)
                               + 1/t_int * int_-inf^t (T_samp(t')-T_set(t')) dt'
                               + t_deriv * dT_samp/dt
        Where P_cout is the percent of the rated max current that the controller
         would like to output if you weren't limiting it,
        T_prop is the propband input to this function, 
        T_samp is the measured temperature of the sample in deg C, 
        T_set is the setpoint in deg C,
        t_int is the integral input to this function,
        the integral with respect to t' is actually only from the time that
         T_samp has been with T_prop of T_set (not -inf), and
        t_deriv is the derivative input to this function.

        Heating is output 2
        """
        check_range(propband, 0, 99.9)
        check_range(integral, 0, 99.99)
        check_range(derivative, 0, 99.99)

    	val = double2melcor(propband)
    	self.write(Melcor.REG_PROPBAND_2, val)
        val = int(integral * 100)
    	self.write(Melcor.REG_INTEGRAL_2, val)
        val = int(derivative * 100)
    	self.write(Melcor.REG_DERIVATIVE_2, val)
    def getHeatingGains(self) :
        "() -> (propband, integral, derivative)"
    	val = self.read(Melcor.REG_PROPBAND_2)
    	propband = melcor2double(val)
    	val = self.read(Melcor.REG_INTEGRAL_2)
        integral = val/100.0
    	val = self.read(Melcor.REG_DERIVATIVE_2)
        derivative = val/100.0
        return (propband, integral, derivative)
    def getFeedbackTerms(self) :
        """
        Experimental
        """
        pid = self.read(Melcor.REG_PID_POWER_1)
        prop = self.read(Melcor.REG_PROP_TERM_1)
    	ntgrl = self.read(Melcor.REG_INTEGRAL_TERM_1)
    	deriv = self.read(Melcor.REG_DERIVATIVE_TERM_1)
        pout = self.getPercentCurrent()
        temp = self.getTemp()
        tset = self.getSetpoint()
        print "pid %g =? sum %g =? cur %g" % (pid, prop+ntgrl+deriv, pout)
        print "meas:     prop %d, integral %d, deriv %d" % (prop, ntgrl, deriv)
        print "my calcs: prop %d" % (temp-tset)
    def setTemp(self, setpoint, tolerance=0.3, time=10.0) :
    	"""
    	Changes setpoint to SETPOINT and waits for stability
    	"""
    	self.setSetpoint(setpoint)
    	while self.isStable(setpoint, tolerance, time) != True :
            pass
    def setTemp_funkygain(self, setpoint, dead_time, heat_rate, cool_rate,
                          peltier_efficiency_fn, outside_equilib_rate,
                          tolerance=0.3, time=10.0) :
        """
        Highly experimental, see diffusion.py
        """
        mode = ""
        T = self.getTemp()
        # full steam ahead
        print "full steam ahead"
    	self.setSetpoint(setpoint)
        self.setHeatingGains(0.1, 0, 0)
        self.setCoolingGains(0.1, 0, 0)
        if T < setpoint :
            mode = "Heating"
            self._heat_until_close(setpoint, dead_time, heat_rate)
        elif T > setpoint :
            mode = "Cooling"
            self._cool_until_close(setpoint, dead_time, cool_rate)
        # coast
        print "coast while temperature equilibrates"
        self.setHeatingGains(100, 0, 0)
        self.setCoolingGains(100, 0, 0)
        time.sleep(dead_time*2)
        cool_prop, heat_prop = self.calcPropBands()
        print "calculated prop bands: c %g, h %g deg C" % (cool_prop, heat_prop)
        print "reset integral gain, and bump to predicted props"
        # pop down to reset integral gain, could also jump setpoint...
        self.setHeatingGains(0.1, 0, 0)
        self.setHeatingGains(heat_prop, 0, 0)
        self.setCoolingGains(0.1, 0, 0)
        self.setCoolingGains(cool_prop, 0, 0)
        time.sleep(dead_time*4)
        # now add in some integral to reduce droop
        print "set integral gains to %g" % (dead_time*4)
        self.setHeatingGains(heat_prop, dead_time*4, 0)
        self.setCoolingGains(cool_prop, dead_time*4, 0)
        time.sleep(dead_time*8)
        print "wait to enter tolerance band"
        while (self.getTemp()-setpoint) :
            time.sleep(dead_time)
        print "should be stable now"
    	if not self.isStable(setpoint, tolerance, time) :
            raise error, "Algorithm broken ;)"
    def _heat_until_close(self, setpoint, dead_time, heat_rate) :
        while self.getTemp() < setpoint - 0.5*rate*dead_time :
            time.sleep(dead_time/10.0)
    def calcPropBands(setpoint, peltier_efficiency_fn, outside_equilib_rate) :
        heat_loss = outside_equilib_rate * (setpoint - self.getAmbientTemp())
        required_current = heat_loss / peltier_efficiency_fn(setpoint)
        if required_current > self.maxCurrent :
            raise errorOutOfRange, "Can't source %g Amps", required_current
        fraction_current = required_current / self.maxCurrent
        droop = 0.5 # expected droop in deg C on only proporitional gain
        # droop / T_prop = fraction current
        T_prop = droop / fraction_current
        if setpoint > self.getAmbientTemp()+5 : # heating
            return (T_prop*10, T_prop)
        elif setpoint < self.getAmbientTemp()+5 : # cooling
            return (T_prop, T_prop*10)
        else : # right about room temperature
            return (T_prop, T_prop)
    def isStable(self, setpoint, tolerance=0.3, maxTime=10.0) :
    	"""
    	Counts how long the temperature stays within
    	TOLERANCE of SETPOINT.
    	Returns when temp goes bad, or MAXTIME elapses.
    	"""
    	stable = False
    	startTime = time.time()
    	stopTime = startTime
    	while abs(self.getTemp() - setpoint) < tolerance :
    		stopTime = time.time()
    		if (stopTime-startTime) > maxTime :
    			print "Stable for long enough"
    			break
    	if stopTime-startTime > maxTime :
    		return True
    	else :
    		return False
    def setFilterTime(self, seconds) :
        """
        Positive values to affect only monitored values.
        Negative values affect both monitored and control values.
        """
    	decSeconds = int(seconds*10)
    	if decSeconds < 0 : # convert (unsigned int) -> (2's compliment signed)
    		decSeconds += 2**16 
    	self.write(Melcor.REG_INPUT_SOFTWARE_FILTER_1, decSeconds)
    def getFilterTime(self) :
        """
        Positive values to affect only monitored values.
        Negative values affect both monitored and control values.
        """
    	val = self.read(Melcor.REG_INPUT_SOFTWARE_FILTER_1)
    	if val >= 2**15 : # convert (2's complement signed) -> (unsigned int)
    		val -= 2**16
    	return val/10.0
    def sanityCheck(self) :
        "Check that some key registers have the values we expect"
    	_sanityCheck(Melcor.REG_UNITS_TYPE,   2) # SI
    	_sanityCheck(Melcor.REG_C_OR_F,       1) # C
    	_sanityCheck(Melcor.REG_FAILURE_MODE, 2) # off
    	_sanityCheck(Melcor.REG_RAMPING_MODE, 0) # off
    	_sanityCheck(Melcor.REG_OUTPUT_1,     1) # cool
    	_sanityCheck(Melcor.REG_OUTPUT_2,     1) # heat
    def _sanityCheck(self, register, expected_value) :
    	val = self.read(register)
    	if val != expected_value :
    		print "Register %d, expected %d, was %d" % (register,
    						    expected_value,
    						    val)
    		raise error, "Controller settings error"
    def read(self, register) :
        """
        (register) -> (value)
        Returns the value of the specified memory register on the controller.
        Registers are defined in the Melcor module.
        See melcor_registers.h for a pointers on meanings and manual page nums.
        """
    	(err, val) = self.T.read(register)
    	if err != 0 :
    		raise errorMelcor
    	return val
    def write(self, register, value) :
        """
        (register, value) -> None
        Sets the value of the specified memory register on the controller.
        Registers are defined in the Melcor module.
        See melcor_registers.h for a pointers on meanings and manual page nums.
        """
    	err = self.T.write(register, value)
    	if err != 0 :
    		raise errorMelcor
    def getDeadtimeData(self, num_oscillations=10, curHysteresis=0.8, log=True) :
        orig_heat_gains = self.getHeatingGains()
        orig_cool_gains = self.getCoolingGains()
        if self.verbose :
            print "Measuring dead time"
            print " go to bang-bang"
        self.setHeatingGains(0.1, 0, 0)
        self.setCoolingGains(0.1, 0, 0)
        def isHeating(cur) :
            if cur > curHysteresis :
                return True
            elif cur < -curHysteresis :
                return False
            else :
                return None
	i=0
	timeArr = [0.0]
	temp = self.getTemp()
	cur = self.getCurrent()
	heat_first = isHeating(cur)
	start_time = time.time()
	tm = 0
        if verbose :
            print " Wait to exit hysteresis region"
	while heat_first == None and tm < 30:
		temp = t.getTemp()
		cur = t.getCurrent()
		heat_first = isHeating(temp, cur)
		tm = time.time()-start_time
	if tm > 30 :
		raise error, "after 30 seconds, still inside hysteresis region"
        if self.verbose :
            print " Read oscillations"
	heating = heat_first
	start_time = time.time()
	tempArr = [temp]
	curArr = [cur]
	if verbose :
		print "Temp %g\t(%g),\tCur %g,\tTime %d" % (temp, temp-Tset, cur, 0)
	while i < numOscillations*2 :
		temp = t.getTemp()
		tm = time.time()-start_time
		cur = t.getCurrent()
		tempArr.append(temp)
		timeArr.append(tm)
		curArr.append(cur)
		check_signs(temp,cur)
		if heating == True and isHeating(temp, cur) == False :
			print "Transition to cooling (i=%d)" % i
			heating = False
			i += 1
		elif heating == False and isHeating(temp, cur) == True :
			print "Transition to heating (i=%d)" % i
			heating = True
			i += 1
        if verbose :
            print " Restoring gains"
        self.setHeatingGains(*orig_heat_gains)
        self.setCoolingGains(*orig_cool_gains)
        if log == True :
            log = 1# MARK

def _test_tempController() :
    t = tempController(controller=1, maxCurrent=0.1)
    
    print "Temp     = %g" % t.getTemp()
    print "Current  = %g" % t.getCurrent()
    print "Setpoint = %g" % t.getSetpoint()
    
    print "Setting setpoint to 5.0 deg C"
    t.setSetpoint(5.0)
    sp = t.getSetpoint()
    print "Setpoint = %g" % sp
    if sp != 5.0 :
        raise Exception, "Setpoint in %g != setpoint out %g" % (sp, 5.0)
    time.sleep(10) # give the controller some time to overcome any integral gain
    c = t.getCurrent() 
    print "Current  = %g" % c
    mca, mcb, mct = t.getMaxCurrent()
    if t.getTemp() < mct : # we're below the high power limit setpoint, use mcb
        if c != mcb :
            raise Exception, "Current not at max %g, and we're shooting for a big temp" % mcb
    else :
        if c != mca :
            raise Exception, "Current not at max %g, and we're shooting for a big temp" % mca

    
    print "Setting setpoint to 50.0 deg C"
    t.setSetpoint(50.0)
    sp = t.getSetpoint()
    print "Setpoint = %g" % sp
    if sp != 5.0 :
        raise Exception, "Setpoint in %g != setpoint out %g" % (sp, 5.0)
    time.sleep(10)
    c = t.getCurrent()
    print "Current  = %g" % c
    print "Success"
    mca, mcb, mct = t.getMaxCurrent()
    if t.getTemp() < mct : # we're below the high power limit setpoint, use mcb
        if -c != mcb :
            raise Exception, "Current not at min %g, and we're shooting for a big temp" % (-mcb)
    else :
        if -c != mca :
            raise Exception, "Current not at min %g, and we're shooting for a big temp" % (-mca)

def test() :
    _test_tempController()

if __name__ == "__main__" :
    test()
