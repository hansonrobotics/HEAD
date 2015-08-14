#!/usr/bin/python3

from setuptools import setup, find_packages
setup(
    name = "roscom-blender-api",
    version = "0.2",
    package_dir = {'': 'src',},
    packages = find_packages('src'),
    entry_points = {
        'blender_api.command_source.build': 'ros = roscom:build'
    }
)
