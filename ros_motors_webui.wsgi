#!/usr/bin/python
from os import path, sys
sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )

from ros_motors_webui.app import app as application
