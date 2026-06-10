#!/usr/bin/env python3
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=[
        "jetracer_autonomous",
        "jetracer_autonomous.control",
        "jetracer_autonomous.decision",
        "jetracer_autonomous.perception",
        "jetracer_autonomous.utils",
        "jetracer_autonomous.vehicle",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
