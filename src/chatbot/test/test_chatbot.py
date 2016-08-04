#!/usr/bin/env python

import unittest
import os
import sys
import atexit
import time
import subprocess

CWD = os.path.abspath(os.path.dirname(__file__))

os.environ['HR_CHARACTER_PATH'] = os.path.join(CWD, 'characters')
server_path = os.path.join(CWD, '../scripts')
PORT = '8002'
cmd = ['python', 'run_server.py', PORT]
proc = subprocess.Popen(cmd, cwd=server_path, preexec_fn=os.setsid)
def shutdown():
    if proc:
        os.killpg(proc.pid, 2)
atexit.register(shutdown)

class ChatbotTest(unittest.TestCase):

    def test_pyaiml(self):
        script = os.path.join(CWD, os.path.sep.join(
            ['..', 'src', 'chatbot', 'aiml', 'Kernel.py']))
        cmd = 'python '+script
        ret = os.system(cmd)
        self.assertTrue(ret==0)

    def test_prologue(self):
        from chatbot.client import Client
        cli = Client('test_client', 'AAAAB3NzaC')
        cli.do_port(PORT)
        while not cli.ping():
            time.sleep(1)
        cli.do_conn('localhost:'+PORT)
        cli.do_select('generic')
        ret, response = cli.ask('hello sophia')
        self.assertTrue(ret == 0)
        self.assertTrue(response.get('text') == 'Hi there from generic')

        cli.do_select('sophia')
        ret, response = cli.ask('hello sophia')
        self.assertTrue(ret == 0)
        self.assertTrue(response.get('text') == 'Hi there from sophia')


if __name__ == '__main__':
    unittest.main()

