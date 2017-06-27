#!/usr/bin/python3

from setuptools import setup, find_packages
setup(
    name = "hr.blender_api_msgs",
    version = "0.3",
    package_dir = {'': 'src',},
    packages = find_packages('src'),
    entry_points = {
        'blender_api.command_source.build': ['ros = roscom:build', 'httpapi = httpapi:build']
    }
)
