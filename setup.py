"""A modular temperature control library.
"""

from distutils.core import setup
import os.path

from tempcontrol import __version__


_this_dir = os.path.dirname(__file__)
base_url = 'http://physics.drexel.edu/~wking'

setup(name='tempcontrol',
      version=__version__,
      maintainer='W. Trevor King',
      maintainer_email='wking@drexel.edu',
      url = '{}/unfolding-disasters/posts/tempcontrol'.format(base_url),
      download_url = '{}/code/python/tempcontrol-{}.tar.gz'.format(
        base_url, __version__),
      license = 'GNU General Public License (GPL)',
      platforms = ['all'],
      description = __doc__,
      long_description = open(os.path.join(_this_dir, 'README'), 'r').read(),
      packages=['tempcontrol', 'tempcontrol.backend'],
      classifiers = [
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Operating System :: OS Independent',
        'License :: OSI Approved :: GNU General Public License (GPL)',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development :: Libraries :: Python Modules',
        ],
      )
