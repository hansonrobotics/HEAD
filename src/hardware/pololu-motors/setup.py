import os, sys, re
from glob import glob
from setuptools import setup

def version():
    regex = r'^(?m){}[\s]*=[\s]*(?P<ver>\d*)$'

    with open(os.path.join(os.path.dirname(__file__), 'include.mk')) as f:
        ver = f.read()

    major = re.search(regex.format('MAJORVERSION'), ver).group('ver')
    minor = re.search(regex.format('MINORVERSION'), ver).group('ver')
    patch = re.search(regex.format('PATCHLEVEL'), ver).group('ver')
    return "{}.{}.{}".format(major, minor, patch)

VERSION = version()

def readme():
    with open(os.path.join(os.path.dirname(__file__), 'README.md')) as f:
        return f.read()

# Allow setup.py to be run from any path.
os.chdir(os.path.normpath(os.path.join(os.path.abspath(__file__), os.pardir)))

setup(
    name='pololu-motors',
    version=VERSION,
    packages=['pololu', 'pololu/motors'],
    include_package_data=True,
    license='MIT',
    description=('Pololu motor driver APIs.'),
    long_description=readme(),
    url='https://github.com/cnobile2012/pololu-motors',
    download_url=('https://github.com/cnobile2012/'
                  'pololu-motors/tarball/t-v{}').format(VERSION),
    author='Carl J. Nobile',
    author_email='carl.nobile@gmail.com',
    keywords='pololu motor API',
    classifiers=[
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 2.7',
        'Topic :: Home Automation',
        'Topic :: System :: Hardware :: Hardware Drivers',
        ],
    install_requires=[
        'pyserial',
        ],
    )
