#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['move_robot'],
    package_dir={'': 'src'},
    scripts=['bin/move_robot_node']
)

setup(**d)
