#!/usr/bin/env python

import os
import sys
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '../src'))

from chatbot.client import Client

if __name__ == '__main__':
    client = Client('client', 'AAAAB3NzaC')
    client.cmdloop()

