import os, sys
from setuptools import setup
from setuptools.command.test import test as TestCommand

README = open(os.path.join(os.path.dirname(__file__), 'README.md')).read()

# allow setup.py to be run from any path
os.chdir(os.path.normpath(os.path.join(os.path.abspath(__file__), os.pardir)))

class Tox(TestCommand):
    user_options = [('tox-args=', 'a', "Arguments to pass to tox")]

    def initialize_options(self):
        TestCommand.initialize_options(self)
        self.tox_args = None

    def finalize_options(self):
        TestCommand.finalize_options(self)
        self.test_args = []
        self.test_suite = True

    def run_tests(self):
        #import here, cause outside the eggs aren't loaded
        import tox
        import shlex
        errno = tox.cmdline(args=shlex.split(self.tox_args))
        sys.exit(errno)


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
    tests_require=['tox'],
    cmdclass = {'test': Tox},
    )
