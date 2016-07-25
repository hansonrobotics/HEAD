#!/usr/bin/env python

import unittest
import os
import sys
import atexit
import time
import subprocess

CWD = os.path.abspath(os.path.dirname(__file__))

server_path = os.path.join(CWD, '../scripts')
PORT = '8002'
cmd = ['python', 'run_server.py', PORT]
proc = subprocess.Popen(cmd, cwd=server_path, preexec_fn=os.setsid)
def shutdown():
    if proc:
        os.killpg(proc.pid, 2)
atexit.register(shutdown)
sys.path.insert(0, os.path.join(CWD, '../src'))

class ChatbotTest(unittest.TestCase):

    def test_pyaiml(self):
        script = os.path.join(CWD, os.path.sep.join(
            ['..', 'src', 'chatbot', 'aiml', 'Kernel.py']))
        cmd = 'python '+script
        ret = os.system(cmd)
        self.assertTrue(ret==0)

    def test_prologue(self):
        from chatbot.client import Client
        cli = Client()
        cli.do_conn('localhost:'+PORT)
        while not cli.ping():
            time.sleep(1)
        cli.do_select('sophia')
        ret, response = cli.ask('what is your name')
        ans = response.get('text')
        names = ['Soepheeyeh', 'Sophia']
        self.assertTrue(any([name in ans for name in names]))

if __name__ == '__main__':
    unittest.main()

