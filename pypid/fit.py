# Copyright (C) 2010-2011 W. Trevor King <wking@drexel.edu>
#
# This file is part of Hooke.
#
# Hooke is free software: you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# Hooke is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General
# Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with Hooke.  If not, see
# <http://www.gnu.org/licenses/>.

"""Provide :class:`ModelFitter` to make arbitrary model fitting easy.
"""

from numpy import arange, ndarray
from scipy import __version__ as _scipy_version
from scipy.optimize import leastsq

_strings = _scipy_version.split('.')
# Don't convert third string to an integer in case of (for example) '0.7.2rc3'
_SCIPY_VERSION = (int(_strings[0]), int(_strings[1]), _strings[2])
del _strings


class PoorFit (ValueError):
    pass

class ModelFitter (object):
    """A convenient wrapper around :func:`scipy.optimize.leastsq`.

    TODO: look into :mod:`scipy.odr` as an alternative fitting
    algorithm (minimizing the sum of squares of orthogonal distances,
    vs. minimizing y distances).

    Parameters
    ----------
    d_data : array_like
        Deflection data to be analyzed for the contact position.
    info :
        Store any extra information useful inside your overridden
        methods.
    rescale : boolean
        Rescale parameters so the guess for each is 1.0.  Also rescale
        the data so data.std() == 1.0.

    Examples
    --------

    >>> from pprint import pprint
    >>> from Queue import Queue
    >>> import numpy

    You'll want to subclass `ModelFitter`, overriding at least
    `.model` and potentially the parameter and scale guessing
    methods.

    >>> class LinearModel (ModelFitter):
    ...     '''Simple linear model.
    ...
    ...     Levenberg-Marquardt is not how you want to solve this problem
    ...     for real systems, but it's a simple test case.
    ...     '''
    ...     def model(self, params):
    ...         '''A linear model.
    ...
    ...         Notes
    ...         -----
    ...         .. math:: y = p_0 x + p_1
    ...         '''
    ...         p = params  # convenient alias
    ...         self._model_data[:] = p[0]*arange(len(self._data)) + p[1]
    ...         return self._model_data
    ...     def guess_initial_params(self, outqueue=None):
    ...         return [float(self._data[-1] - self._data[0])/len(self._data),
    ...                 self._data[0]]
    ...     def guess_scale(self, params, outqueue=None):
    ...         slope_scale = 0.1
    ...         if params[1] == 0:
    ...             offset_scale = 1
    ...         else:
    ...             offset_scale = 0.1*self._data.std()/abs(params[1])
    ...             if offset_scale == 0:  # data is completely flat
    ...                 offset_scale = 1.
    ...         return [slope_scale, offset_scale]
    >>> data = 20*numpy.sin(arange(1000)) + 7.*arange(1000) - 33.0
    >>> m = LinearModel(data)
    >>> outqueue = Queue()
    >>> slope,offset = m.fit(outqueue=outqueue)
    >>> info = outqueue.get(block=False)
    >>> pprint(info)  # doctest: +ELLIPSIS, +NORMALIZE_WHITESPACE, +REPORT_UDIFF
    {'active fitted parameters': array([  6.999..., -32.889...]),
     'active parameters': array([  6.999..., -32.889...]),
     'convergence flag': ...,
     'covariance matrix': array([[  1.199...e-08,  -5.993...e-06],
           [ -5.993...e-06,   3.994...e-03]]),
     'data scale factor': 1.0,
     'fitted parameters': array([  6.999..., -32.889...]),
     'info': {'fjac': array([[...]]),
              'fvec': array([...]),
              'ipvt': array([1, 2]),
              'nfev': 7,
              'qtf': array([...])},
     'initial parameters': [6.992..., -33.0],
     'message': '...relative error between two consecutive iterates is at most 0.000...',
     'rescaled': False,
     'scale': [0.100..., 6.123...]}

    We round the outputs to protect the doctest against differences in
    machine rounding during computation.  We expect the values to be close
    to the input settings (slope 7, offset -33).

    >>> print '%.3f' % slope
    7.000
    >>> print '%.3f' % offset
    -32.890

    The offset is a bit off because, the range is not a multiple of
    :math:`2\pi`.

    We could also use rescaled parameters:

    >>> m = LinearModel(data, rescale=True)
    >>> outqueue = Queue()
    >>> slope,offset = m.fit(outqueue=outqueue)
    >>> print '%.3f' % slope
    7.000
    >>> print '%.3f' % offset
    -32.890

    Test single-parameter models:

    >>> class SingleParameterModel (LinearModel):
    ...     '''Simple linear model.
    ...     '''
    ...     def model(self, params):
    ...         return super(SingleParameterModel, self).model([params[0], 0.])
    ...     def guess_initial_params(self, outqueue=None):
    ...         return super(SingleParameterModel, self
    ...             ).guess_initial_params(outqueue)[:1]
    ...     def guess_scale(self, params, outqueue=None):
    ...         return super(SingleParameterModel, self
    ...             ).guess_scale([params[0], 0.], outqueue)[:1]
    >>> data = 20*numpy.sin(arange(1000)) + 7.*arange(1000)
    >>> m = SingleParameterModel(data)
    >>> slope, = m.fit(outqueue=outqueue)
    >>> print '%.3f' % slope
    7.000
    """
    def __init__(self, *args, **kwargs):
        self.set_data(*args, **kwargs)

    def set_data(self, data, info=None, rescale=False):
        self._data = data
        self._model_data = ndarray(shape=data.shape, dtype=data.dtype)
        self.info = info
        self._rescale = rescale
        if rescale == True:
            for x in [data.std(), data.max()-data.min(), abs(data.max()), 1.0]:
                if x != 0:
                    self._data_scale_factor = x
                    break
        else:
            self._data_scale_factor = 1.0

    def model(self, params):
        p = params  # convenient alias
        self._model_data[:] = arange(len(self._data))
        raise NotImplementedError

    def guess_initial_params(self, outqueue=None):
        return []

    def guess_scale(self, params, outqueue=None):
        """Guess the problem length scale in each parameter dimension.

        Notes
        -----
        From the :func:`scipy.optimize.leastsq` documentation, `diag`
        (which we refer to as `scale`, sets `factor * || diag * x||`
        as the initial step.  If `x == 0`, then `factor` is used
        instead (from `lmdif.f`_)::

                      do 70 j = 1, n
                        wa3(j) = diag(j)*x(j)
               70       continue
                      xnorm = enorm(n,wa3)
                      delta = factor*xnorm
                      if (delta .eq. zero) delta = factor
  
        For most situations then, you don't need to do anything fancy.
        The default scaling (if you don't set a scale) is::

            c        on the first iteration and if mode is 1, scale according
            c        to the norms of the columns of the initial jacobian.

        (i.e. `diag(j) = acnorm(j)`, where `acnorm(j) is the norm of the `j`th column
        of the initial Jacobian).

        .. _lmdif.f: http://www.netlib.org/minpack/lmdif.f
        """
        return None

    def residual(self, params):
        if self._rescale == True:
            params = [p*s for p,s in zip(params, self._param_scale_factors)]
        residual = self._data - self.model(params)
        if False:  # fit debugging
            if not hasattr(self, '_i_'):
                self._i_ = 0
            self._data.tofile('data.%d' % self._i_, sep='\n')
            self.model(params).tofile('model.%d' % self._i_, sep='\n')
            self._i_ += 1
        if self._rescale == True:
            residual /= self._data_scale_factor
        return residual

    def fit(self, initial_params=None, scale=None, outqueue=None, **kwargs):
        """
        Parameters
        ----------
        initial_params : iterable or None
            Initial parameter values for residual minimization.  If
            `None`, they are estimated from the data using
            :meth:`guess_initial_params`.
        scale : iterable or None
            Parameter length scales for residual minimization.  If
            `None`, they are estimated from the data using
            :meth:`guess_scale`.
        outqueue : Queue or None
            If given, will be used to output the data and fitted model
            for user verification.
        kwargs :
            Any additional arguments are passed through to `leastsq`.
        """
        if initial_params == None:
            initial_params = self.guess_initial_params(outqueue)
        if scale == None:
            scale = self.guess_scale(initial_params, outqueue)
        if scale != None:
            assert min(scale) > 0, scale
        if self._rescale == True:
            self._param_scale_factors = initial_params
            for i,s in enumerate(self._param_scale_factors):
                if s == 0:
                    self._param_scale_factors[i] = 1.0
            active_params = [p/s for p,s in zip(initial_params,
                                                self._param_scale_factors)]
        else:
            active_params = initial_params
        params,cov,info,mesg,ier = leastsq(
            func=self.residual, x0=active_params, full_output=True,
            diag=scale, **kwargs)
        if len(initial_params) == 1 and _SCIPY_VERSION < (0, 8, '0'):
            # params is a float for scipy < 0.8.0.  Convert to list.
            params = [params]
        if self._rescale == True:
            active_params = params
            params = [p*s for p,s in zip(params,
                                         self._param_scale_factors)]
        else:
            active_params = params
        if outqueue != None:
            outqueue.put({
                    'rescaled': self._rescale,
                    'initial parameters': initial_params,
                    'active parameters': active_params,
                    'scale': scale,
                    'data scale factor': self._data_scale_factor,
                    'active fitted parameters': active_params,
                    'fitted parameters': params,
                    'covariance matrix': cov,
                    'info': info,
                    'message': mesg,
                    'convergence flag': ier,
                    })
        return params

# Example ORD code from the old fit.py
#        def dist(px,py,linex,liney):
#            distancesx=scipy.array([(px-x)**2 for x in linex])
#            minindex=numpy.argmin(distancesx)
#            print px, linex[0], linex[-1]
#            return (py-liney[minindex])**2
#
#
#        def f_wlc(params,x,T=T):
#            '''
#            wlc function for ODR fitting
#            '''
#            lambd,pii=params
#            Kb=(1.38065e-23)
#            therm=Kb*T
#            y=(therm*pii/4.0) * (((1-(x*lambd))**-2) - 1 + (4*x*lambd))
#            return y
#
#        def f_wlc_plfix(params,x,pl_value=pl_value,T=T):
#            '''
#            wlc function for ODR fitting
#            '''
#            lambd=params
#            pii=1/pl_value
#            Kb=(1.38065e-23)
#            therm=Kb*T
#            y=(therm*pii/4.0) * (((1-(x*lambd))**-2) - 1 + (4*x*lambd))
#            return y
#
#        #make the ODR fit
#        realdata=scipy.odr.RealData(xchunk_corr_up,ychunk_corr_up)
#        if pl_value:
#            model=scipy.odr.Model(f_wlc_plfix)
#            o = scipy.odr.ODR(realdata, model, p0_plfix)
#        else:
#            model=scipy.odr.Model(f_wlc)
#            o = scipy.odr.ODR(realdata, model, p0)
#
#        o.set_job(fit_type=2)
#        out=o.run()
#        fit_out=[(1/i) for i in out.beta]
#
#        #Calculate fit errors from output standard deviations.
#        #We must propagate the error because we fit the *inverse* parameters!
#        #The error = (error of the inverse)*(value**2)
#        fit_errors=[]
#        for sd,value in zip(out.sd_beta, fit_out):
#            err_real=sd*(value**2)
#            fit_errors.append(err_real)

