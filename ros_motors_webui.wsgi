#!/usr/bin/python

activate_this = '/opt/ros_motors_webui/venv/bin/activate_this.py'
execfile(activate_this, dict(__file__=activate_this))

import sys
import logging
logging.basicConfig(stream=sys.stderr)
sys.path.insert(0,"/opt/ros_motors_webui/app")

from app import app as application
