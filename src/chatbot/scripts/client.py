#!/usr/bin/env python

import os
import sys
import logging
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '../src'))

from chatbot.client import Client

HR_CHATBOT_AUTHKEY = os.environ.get('HR_CHATBOT_AUTHKEY', 'AAAAB3NzaC')

if __name__ == '__main__':
    logging.basicConfig()
    logging.getLogger().setLevel(logging.WARN)
    if len(sys.argv)>1:
        client = Client(HR_CHATBOT_AUTHKEY, botname=sys.argv[1])
    else:
        client = Client(HR_CHATBOT_AUTHKEY)
    client.cmdloop()
