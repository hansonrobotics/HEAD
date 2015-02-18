import os, sys
from setuptools import setup
from setuptools.command.test import test as TestCommand

README = open(os.path.join(os.path.dirname(__file__), 'README.md')).read()

# allow setup.py to be run from any path
os.chdir(os.path.normpath(os.path.join(os.path.abspath(__file__), os.pardir)))

setup(
    name='pololu-motors',
    version='1.0.0',
    packages=['motors',],
    include_package_data=True,
    license='MIT License',
    description=('Pololu motor drivers APTs.'),
    long_description=README,
    url='https://github.com/cnobile2012/pololu_motors',
    author='Carl J. Nobile',
    author_email='carl.nobile@gmail.com',
    classifiers=[
        'Intended Audience :: Raspberry Pi, Beagle Bone',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Topic :: Home Automation :: Motor Control API :: Pololu Qik 2s9v1',
        ],
    )
