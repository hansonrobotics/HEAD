#!/usr/bin/env python

import os
import sys
import logging
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '../src'))

from chatbot.client import Client
from chatbot.client import get_default_username

HR_CHATBOT_AUTHKEY = os.environ.get('HR_CHATBOT_AUTHKEY', 'AAAAB3NzaC')

if __name__ == '__main__':
    logging.basicConfig()
    logging.getLogger().setLevel(logging.WARN)
    client = Client(get_default_username(), HR_CHATBOT_AUTHKEY)
    client.cmdloop()
