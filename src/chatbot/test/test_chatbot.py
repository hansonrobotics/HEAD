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

from chatbot.client import Client

def shutdown():
    if proc:
        os.killpg(proc.pid, 2)
atexit.register(shutdown)


class ChatbotTest(unittest.TestCase):

    def test_pyaiml(self):
        script = os.path.join(CWD, os.path.sep.join(
            ['..', 'src', 'chatbot', 'aiml', 'Kernel.py']))
        cmd = 'python ' + script
        ret = os.system(cmd)
        self.assertTrue(ret == 0)

    def test_prologue(self):
        cli = Client('AAAAB3NzaC', username='test_client', port=PORT, test=True)
        while not cli.ping():
            time.sleep(1)
        cli.do_conn('localhost:' + PORT)
        cli.do_select('generic')
        response = cli.ask('hello sophia')
        self.assertTrue(response.get('text') == 'Hi there from generic')

        cli.do_select('sophia')
        response = cli.ask('hello sophia')
        self.assertTrue(response.get('text') == 'Hi there from sophia')

    def test_two_clients(self):
        cli = Client('AAAAB3NzaC', botname='generic', port=PORT, test=True)
        cli2 = Client('AAAAB3NzaC', botname='sophia', port=PORT, test=True)
        while not cli.ping():
            time.sleep(1)
        cli.do_conn('localhost:' + PORT)
        cli2.do_conn('localhost:' + PORT)
        response = cli.ask('hello sophia')
        self.assertTrue(response.get('text') == 'Hi there from generic')

        response = cli2.ask('hello sophia')
        self.assertTrue(response.get('text') == 'Hi there from sophia')

    def test_session_manager(self):
        from chatbot.server.session import SessionManager
        session_manager = SessionManager(False)
        sid = session_manager.start_session(user='test', key='key', test=True)
        session = session_manager.get_session(sid)
        self.assertIsNotNone(session)
        self.assertIsNone(session.cache.last_time)

        self.assertTrue(session.add("hi", "hi there"))
        self.assertIsNotNone(session.cache.last_time)

        session_manager.reset_session(sid)
        self.assertIsNotNone(session)
        self.assertIsNone(session.cache.last_time)

        session_manager.remove_session(sid)
        self.assertFalse(session.add("hi", "hi there"))
        session = session_manager.get_session(sid)
        self.assertIsNone(session)

    def test_session_manager_auto(self):
        import chatbot.server.config
        chatbot.server.config.SESSION_REMOVE_TIMEOUT = 2
        from chatbot.server.session import SessionManager
        reload(chatbot.server.session)

        session_manager = SessionManager(True)
        sid = session_manager.start_session(user='test', key='key', test=True)
        session = session_manager.get_session(sid)
        self.assertIsNotNone(session)
        self.assertIsNone(session.cache.last_time)

        time.sleep(0.5)

        # session cache should have record
        self.assertTrue(session.add("hi", "hi there"))
        self.assertIsNotNone(session.cache.last_time)

        # session should not be removed
        time.sleep(1)
        self.assertIsNotNone(session.cache.last_time)

        # session should be removed
        time.sleep(1.5)
        self.assertFalse(session.add("hi", "hi there"))
        session = session_manager.get_session(sid)
        self.assertIsNone(session)

    def test_chat_agent(self):
        from chatbot.server.chatbot_agent import session_manager, ask
        sid = session_manager.start_session(user='test', key='key', test=True)
        sess = session_manager.get_session(sid)
        sess.sdata.botname = 'sophia'
        sess.sdata.user = 'test'
        response, ret = ask('what is your name', 'en', sid)
        self.assertEqual(ret, 0)
        response, ret = ask('', 'en', sid)
        self.assertEqual(ret, 4)
        response, ret = ask(None, 'en', sid)
        self.assertEqual(ret, 4)

    def test_loader(self):
        from chatbot.server.loader import load_characters
        character_path = os.environ.get('HR_CHARACTER_PATH')
        characters = load_characters(character_path)
        names = [character.name for character in characters]
        self.assertEqual(names, ['dummy', 'generic', 'sophia'])

if __name__ == '__main__':
    unittest.main()
