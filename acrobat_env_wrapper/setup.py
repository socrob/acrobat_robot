#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['acrobat_env_wrapper_ros'],
 package_dir={'acrobat_env_wrapper_ros': 'ros/src/acrobat_env_wrapper_ros'}
)

setup(**d)
