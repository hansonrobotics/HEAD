#!/usr/bin/env python

import unittest
import os
import sys
import time
import subprocess
import signal

RCFILE = os.environ.get('COVERAGE_RCFILE', '.coveragerc')

from chatbot.client import Client

class ChatbotTest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.port = '8002'
        self.cwd = os.path.abspath(os.path.dirname(__file__))
        os.environ['HR_CHARACTER_PATH'] = os.path.join(self.cwd, 'characters')
        server = os.path.join(self.cwd, '../scripts/run_server.py')
        cmd = ['coverage', 'run', '--rcfile', RCFILE, server, '-p', self.port]
        self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)

    @classmethod
    def tearDownClass(self):
        if self.proc:
            while self.proc.poll() is None:
                try:
                    print "Shuting down"
                    self.proc.send_signal(signal.SIGINT)
                except OSError as ex:
                    print ex
                time.sleep(0.2)
            try:
                os.killpg(self.proc.pid, 2)
            except OSError as ex:
                print "Killed"

    def test_pyaiml(self):
        script = os.path.join(self.cwd, os.path.sep.join(
            ['..', 'src', 'chatbot', 'aiml', 'Kernel.py']))
        cmd = 'python ' + script
        ret = os.system(cmd)
        self.assertTrue(ret == 0)

    def test_prologue(self):
        cli = Client('AAAAB3NzaC', username='test_client', port=self.port, test=True)
        while not cli.ping():
            time.sleep(1)
        cli.do_conn('localhost:' + self.port)
        cli.do_select('generic')
        response = cli.ask('hello sophia')
        self.assertTrue(response.get('text') == 'Hi there from generic')

        cli.do_select('sophia')
        response = cli.ask('hello sophia')
        self.assertTrue(response.get('text') == 'Hi there from sophia')

    def test_two_clients(self):
        cli = Client('AAAAB3NzaC', botname='generic', port=self.port, test=True)
        cli2 = Client('AAAAB3NzaC', botname='sophia', port=self.port, test=True)
        while not cli.ping():
            time.sleep(1)
        cli.do_conn('localhost:' + self.port)
        cli2.do_conn('localhost:' + self.port)
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

    def test_polarity(self):
        from chatbot.polarity import Polarity
        p = Polarity()
        p.load_sentiment_csv(os.path.join(
                    self.cwd, '../scripts/aiml/senticnet3.props.csv'))
        self.assertTrue(p.get_polarity("The dish is yucky") < 0)
        self.assertTrue(p.get_polarity("The weather is nice") > 0)

    def test_util(self):
        import chatbot.utils as utils
        text = '''My mind is built using Hanson Robotics' character engine, a simulated humanlike brain that runs inside a personal computer. Within this framework, Hanson has modelled Phil's personality and emotions, allowing you to talk with Phil through me, using speech recognition, natural language understanding, and computer vision such as face recognition, and animation of the robotic muscles in my face.'''
        text2 = utils.shorten(text, 123)
        self.assertTrue(len(text2) <= 123)
        text2 = utils.shorten(text, 0)
        self.assertTrue(len(text2) > 0)
        self.assertTrue(utils.str_cleanup(' . ') == '')
        self.assertTrue(utils.str_cleanup(' .ss ') == 'ss')
        self.assertTrue(utils.str_cleanup(' s.ss ') == 's.ss')
        self.assertTrue(utils.str_cleanup(None) is None)
        self.assertTrue(utils.check_online('google.com'))
        self.assertTrue(utils.check_online('google.com', 80))
        self.assertTrue(not utils.check_online('google.com', 81))

    def test_words2num(self):
        from chatbot.words2num import words2num
        self.assertTrue(words2num('one hundred trillion and twelve') == 100000000000012)
        self.assertTrue(words2num('one hundred trillion twelve hundred and 21') == 100000000001221)
        self.assertTrue(words2num("one hundred and seventy nine") == 179)
        self.assertTrue(words2num("thirteen hundred") == 1300)
        self.assertTrue(words2num("nine thousand two hundred and ninety seven") == 9297)
        self.assertTrue(words2num(None) is None)
        self.assertTrue(words2num("zero") == 0)

if __name__ == '__main__':
    unittest.main()
