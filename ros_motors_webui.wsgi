#!/usr/bin/python3
from os import path, sys
sys.path.append( path.dirname( path.dirname( path.abspath(__file__) + "/app" ) ) )

from app import app as application
