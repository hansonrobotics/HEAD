# Pololu Motor Control API

## Installation

It is often useful to install python's virtual environments on your system. 
If using a Raspberry Pi, Beagle Bone, or any other Linux based credit card
sized computer a virtual environment, can be useful, but is not necessary. On 
a larger computer it can help keep different packages separated from each 
other.

### Installing a Virtual Environment
If you choose to use virtual environments in your working environment the 
following steps will install it for you. The docs are here: 
[virtualenvwrapper](https://virtualenvwrapper.readthedocs.org/en/latest/).

  * This will install the setuptools package needed to get `easy_install`.
    * `$ sudo apt-get install python-setuptools`
  * Then use `easy_install` to install `pip`.
    * `$ sudo easy_install pip`
  * Then use `pip` to install the virtual environment.
    * `$ sudo pip install virtualenvwrapper`
  * Add the next two lines, using your favorite text editor, to the end of the
    `.bashrc` file in your home directory. 
    * ```
      # Setup the Python virtual environment.
      source /usr/local/bin/virtualenvwrapper.sh```
  * Lastly resource the `.bashrc` file in your home directory.
    * `$ source ~/.bashrc`

Virtual Environment Hints:
There are many commands that can be used with a virtual environment, for a 
full list see the link above. All commands below assume the virtual 
environment will be named `pololu`.

  * To create a new environment type the below command outside of any virtual 
    environments you may already have.
    * `$ mkvirtualenv pololu`
  * To enter an environment.
    * `$ workon pololu
  * To exit a virtual environment.
    * `$ deactivate`
  * To completely remove a virtual environment.
    * `$ rmvirtualenv pololu`

### Installing with pip

Installing via `pip` is probably the simplest, but may not work on all 
platforms. If using a virtual environment be sure you are in the virtual 
environment before running the below command. Documentation for pip can be
found at [pip](https://pip.pypa.io/en/latest/)

  * `pip install pololu-motors`

That's it, there is no more to do. The package will also install HTML API docs 
in <prefix>/share/doc/pololu_motors/index.html. Where <prefix> can be 
`/usr`, `/usr/local`, or `~/.virtualenvs/pololu`.

### Installing with setup.py

