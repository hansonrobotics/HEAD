import os, sys
from glob import glob
from setuptools import setup

def readme():
    with open(os.path.join(os.path.dirname(__file__), 'README.md')) as f:
        return f.read()

# allow setup.py to be run from any path
os.chdir(os.path.normpath(os.path.join(os.path.abspath(__file__), os.pardir)))

version = '0.1.1'

setup(
    name='pololu-motors',
    version=version,
    packages=['motors', 'docs/pmcapi',],
    data_files=[(os.path.join(
        sys.prefix, 'share', 'doc', 'pololu_motors'), glob("docs/pmcapi/*"))],
    include_package_data=True,
    license='MIT',
    description=('Pololu motor driver APIs.'),
    long_description=readme(),
    url='https://github.com/cnobile2012/pololu-motors',
    download_url=('https://github.com/cnobile2012/'
                  'pololu-motors/tarball/t-v{}').format(version),
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
    )
