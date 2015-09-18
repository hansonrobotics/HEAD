# Pololu Motor Control API

## Installation

It is often useful to install python's virtual environments on your system. 
If using a Raspberry Pi, Beagle Bone, or any other Linux based credit card
sized computer a virtual environment, can be useful, but is not necessary. On 
a larger computer it can help keep different project's packages separate from 
each other.

### Installing a Virtual Environment

If you choose to use virtual environments for your working environment the 
following steps will install it for you. The docs are at: 
[virtualenvwrapper](https://virtualenvwrapper.readthedocs.org/en/latest/).

  * This will install the setuptools package needed to get `easy_install`.
    * `$ sudo apt-get install python-setuptools`
  * Then use `easy_install` to install `pip`.
    * `$ sudo easy_install pip`
  * Then use `pip` to install the virtual environment.
    * `$ sudo pip install virtualenvwrapper`
  * Add the next two lines, using your favorite text editor, to the end of the
    `.bashrc` file in your home directory. 
    * ```# Setup the Python virtual environment.
         source /usr/local/bin/virtualenvwrapper.sh```
  * Lastly resource the `.bashrc` file in your home directory.
    * `$ source ~/.bashrc`

#### Virtual Environment Hints

There are many commands that can be used with a virtual environment, for a 
full list see the link above. All commands below assume the virtual 
environment will be named `pololu`.

  * To create a new environment type the below command outside of any virtual 
    environments you may already have.
    * `$ mkvirtualenv pololu`
  * To enter a virtual environment.
    * `$ workon pololu
  * To exit a virtual environment.
    * `$ deactivate`
  * To completely remove a virtual environment.
    * `$ rmvirtualenv pololu`

### Installing `pololu-motors` with pip

Installing via `pip` is probably the simplest, but may not work on all 
platforms. If using a virtual environment be sure you are in the virtual 
environment before running the below command. Documentation for pip can be
found at: [pip](https://pip.pypa.io/en/latest/).

  * `$ pip install pololu-motors`

or directly from the git repository

  * Check the version number at the end of the URL, it may have changed.
    * `$ pip install git+https://github.com/cnobile2012/pololu-motors/t-v0.2.1`

That's it, there is no more to do. The package will also install HTML API docs 
in `<prefix>/share/doc/pololu_motors/index.html`. Where `<prefix>` can be 
`/usr`, `/usr/local`, or `~/.virtualenvs/pololu`.

### Installing `pololu-motors` with setup.py

Installing this way is best if you get the code from a compressed file like a 
`.zip` or `.tar.gz`. This also would work from a clone of the git repository 
at: [pololu-motors](https://github.com/cnobile2012/pololu-motors).

  * Search on [PyPi](https://pypi.python.org/) for `pololu-motors` then 
    download the file.

or

  * Clone the GitHub repository with 
    * `$ git clone https://github.com/cnobile2012/pololu-motors.git`

After you have the package and expanded it in your work area, you may want to 
install it. Installing the package is not necessary if you just want to test 
it out, you can do that directly from the the git clone or the directory 
created by the downloaded tarball.

  * This will install `pololu-motors` globally.
    * `$ sudo python setup.py install`

## Building the Documentation

The API docs are generated with [epydoc](http://epydoc.sourceforge.net/). If 
you want to build the docs yourself there are a few packages that need to be 
installed globally on your machine.

  * Install the necessary packages.
    * `$ sudo apt-get install python-epydoc epydoc-doc graphviz`
  * Build the docs.
    * `$ make api-docs`

## Running unit tests.

Any good package should have unit tests. These tests give the developer a 
baseline of how the code should perform. To run the tests you will need a
[Qik 2s9v1](https://www.pololu.com/product/1110), 
[USB to serial adapter](https://www.pololu.com/product/1308),  
[USB Cable A to Micro-B](https://www.pololu.com/product/1938), 
breadboard and some jumper leads to connect it all together.

After all these items are installed properly you can run the tests. You can 
also connect a motor to M0. M1 is not tested at this time since most of the 
code is tested with M0.

There are two ways to run the unit tests, with the `Makefile` directly or 
with `tox`.

### Using the `Makefile`

  * `$ make tests`

### Using `tox`

First you will need to install the tox subsystem then you will be able to run 
the tests.

  * This command will install `tox` in a Python 3.x directory on your system, 
    but this will not matter since it is run independently from your 
    `pololu-motor` install.
    * `$ sudo pip install tox`
  * Now run `tox`. It will run all the tests.
    * `$ tox`

## Build Instructions

  * The package is built with the `make dist` target in the `Makefile`.
  * The version number of the package is <major>.<minor>.<patch>` and is only
    changed in the `include.mk` file. You will not find the version defined 
    anywhere else.


--------------------------------------------------------------------
If you have any issue please contact me at: carl.nobile at gmail.com
