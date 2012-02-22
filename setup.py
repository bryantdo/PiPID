"""A modular PID control library.
"""

from distutils.core import setup
import os.path

from pypid import __version__


_this_dir = os.path.dirname(__file__)

setup(name='pypid',
      version=__version__,
      maintainer='W. Trevor King',
      maintainer_email='wking@drexel.edu',
      url='http://blog.tremily.us/posts/pypid',
      download_url='http://git.tremily.us/?p=pypid.git;a=snapshot;h=v{};sf=tgz'.format(
        __version__),
      license = 'GNU General Public License (GPL)',
      platforms = ['all'],
      description = __doc__,
      long_description = open(os.path.join(_this_dir, 'README'), 'r').read(),
      packages=['pypid', 'pypid.backend'],
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
