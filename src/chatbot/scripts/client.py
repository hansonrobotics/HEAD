#!/usr/bin/env python

import os
import sys
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '../src'))

from chatbot.client import Client
from chatbot.client import get_default_username

if __name__ == '__main__':
    client = Client(get_default_username(), 'AAAAB3NzaC')
    client.cmdloop()
