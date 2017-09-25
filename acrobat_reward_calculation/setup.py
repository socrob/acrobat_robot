#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['acrobat_reward_calculation', 'acrobat_reward_calculation_ros'],
 package_dir={'acrobat_reward_calculation': 'common/src/acrobat_reward_calculation', 'acrobat_reward_calculation_ros': 'ros/src/acrobat_reward_calculation_ros'}
)

setup(**d)
