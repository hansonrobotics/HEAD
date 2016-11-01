import cmd
import requests
import json
import os
import re
import time
from functools import wraps
import logging
import threading
from chatbot.utils import norm

import sys
reload(sys)
sys.setdefaultencoding('utf-8')
VERSION = 'v1.1'

logger = logging.getLogger('hr.chatbot.client')

def get_default_username():
    import subprocess
    user = subprocess.check_output('id -nu', shell=True).strip()
    host = subprocess.check_output('hostname', shell=True).strip()
    return '{}@{}'.format(user, host)


class Client(cmd.Cmd, object):

    def __init__(self, key, response_listener=None, username=None, botname='sophia',
            host='localhost', port='8001', test=False,
            *args, **kwargs):
        """
        key: The authentication key for chatbot server.
        response_listener: The object that has implemented on_response.
        username: The user name.
        botname: The bot name.
        host: The host name of chatbot server.
        port: The port of the host.
        test: If the session is a test session.
        """
        super(Client, self).__init__(*args, **kwargs)
        self.user = username or get_default_username()
        self.key = key
        if response_listener:
            assert hasattr(response_listener, 'on_response') and \
                callable(response_listener.on_response)
        self.response_listener = response_listener
        self.test = test
        self.prompt = '[me]: '
        self.botname = botname
        self.chatbot_ip = host
        self.chatbot_port = port
        self.chatbot_url = 'http://{}:{}'.format(
            self.chatbot_ip, self.chatbot_port)
        self.root_url = '{}/{}'.format(self.chatbot_url, VERSION)
        self.lang = 'en'
        self.session = None
        self.last_response = None
        self.timer = None
        self.timeout = None
        if self.ping():
            self.do_conn()
        else:
            self.stdout.write(
                "Chatbot server is not responding. Server url {}\n".format(self.chatbot_url))

    def retry(times):
        def wrap(f):
            @wraps(f)
            def wrap_f(*args):
                for i in range(times):
                    try:
                        return f(*args)
                    except Exception as ex:
                        logger.error(ex)
                        self = args[0]
                        self.start_session()
                        continue
            return wrap_f
        return wrap

    def start_session(self, new=False):
        params = {
            "Auth": self.key,
            "botname": self.botname,
            "user": self.user,
            "test": self.test,
            "refresh": new
        }
        response = None
        try:
            response = requests.get(
                '{}/start_session'.format(self.root_url), params=params)
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))
        if response is None:
            self.stdout.write(
                "Can't get session\nPlease check the url {}\n".format(self.chatbot_url))
            return
        if response.status_code != 200:
            self.stdout.write("Request error: {}\n".format(response.status_code))
            return
        session = response.json().get('sid')
        if self.session == session:
            self.stdout.write("Resume session {}\n".format(self.session))
        else:
            self.session = session
            self.stdout.write("Init session {}\n".format(self.session))

    @retry(3)
    def ask(self, question, query=False):
        self.cancel_timer()
        params = {
            "question": "{}".format(question),
            "session": self.session,
            "lang": self.lang,
            "Auth": self.key,
            "query": query
        }
        r = requests.get('{}/chat'.format(self.root_url), params=params)
        ret = r.json().get('ret')
        if r.status_code != 200:
            self.stdout.write("Request error: {}\n".format(r.status_code))

        if ret != 0:
            self.stdout.write("QA error: error code {}, botname {}, question {}, lang {}\n".format(
                ret, self.botname, question, self.lang))
            raise Exception("QA error code {}".format(ret))

        response = {'text': '', 'emotion': '', 'botid': '', 'botname': ''}
        response.update(r.json().get('response'))

        if question == '[loopback]':
            self.timer = threading.Timer(self.timeout, self.ask, (question, ))
            self.timer.start()
            logger.info("Start {} timer with timeout {}".format(
                question, self.timeout))

        self.process_response(response)
        return response

    def list_chatbot(self):
        params = {'Auth': self.key, 'lang': self.lang, 'session': self.session}
        r = requests.get(
            '{}/chatbots'.format(self.root_url), params=params)
        chatbots = r.json().get('response')
        return chatbots

    def list_chatbot_names(self):
        params = {'Auth': self.key, 'lang': self.lang, 'session': self.session}
        r = requests.get(
            '{}/bot_names'.format(self.root_url), params=params)
        names = r.json().get('response')
        return names

    def process_response(self, response):
        if response is not None:
            answer = response.get('text')
            self.process_indicator(answer)
            response['text'] = norm(answer)
            self.last_response = response
            if self.response_listener is None:
                self.stdout.write('{}[by {}]: {}\n'.format(
                    self.botname, response.get('botid'),
                    response.get('text')))
            else:
                try:
                    self.response_listener.on_response(self.session, response)
                except Exception as ex:
                    logger.error(ex)

    def cancel_timer(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            logger.info("Timer canceled")
        else:
            logger.info("Timer is None")

    def default(self, line):
        try:
            if line:
                self.ask(line)
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def do_list(self, line):
        chatbots = []
        try:
            chatbots = self.list_chatbot()
            chatbots = ['{}: weight: {} level: {} dynamic level: {}'.format(
                c, w, l, d) for c, w, l, d in chatbots]
            self.stdout.write('\n'.join(chatbots))
            self.stdout.write('\n')
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def help_list(self):
        self.stdout.write("List chatbot names\n")

    do_l = do_list
    help_l = help_list

    def do_select(self, line):
        try:
            names = self.list_chatbot_names()
            if line in names:
                self.botname = line
                self.start_session()
                self.stdout.write("Select chatbot {}\n".format(self.botname))
            else:
                self.stdout.write("No such chatbot {}\n".format(line))
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))
            return

    def help_select(self):
        self.stdout.write("Select chatbot\n")

    def do_conn(self, line=None):
        if line:
            try:
                self.chatbot_ip, self.chatbot_port = line.split(':')
                self.chatbot_url = 'http://{}:{}'.format(
                    self.chatbot_ip, self.chatbot_port)
                self.root_url = '{}/{}'.format(self.chatbot_url, VERSION)
            except Exception:
                self.stdout.write("Wrong conn argument\n")
                self.help_conn()
                return
        self.stdout.write("Connecting {}\n".format(self.chatbot_url))
        self.start_session()

    def help_conn(self):
        s = """
Connect to chatbot server
Syntax: conn [url:port]
For example, conn
             conn 127.0.0.1:8001

"""
        self.stdout.write(s)

    def do_ip(self, line):
        self.chatbot_ip = line
        self.chatbot_url = 'http://{}:{}'.format(
            self.chatbot_ip, self.chatbot_port)
        self.root_url = '{}/{}'.format(self.chatbot_url, VERSION)

    def help_ip(self):
        s = """
Set the IP address of chatbot server
Syntax: ip xxx.xxx.xxx.xxx
For example, ip 127.0.0.1

"""
        self.stdout.write(s)

    def do_port(self, line):
        self.chatbot_port = line
        self.chatbot_url = 'http://{}:{}'.format(
            self.chatbot_ip, self.chatbot_port)
        self.root_url = '{}/{}'.format(self.chatbot_url, VERSION)

    def help_port(self):
        s = """
Set the port of chatbot server
Syntax: port xxx
For example, port 8001

"""
        self.stdout.write(s)

    def do_q(self, line):
        self.stdout.write("Bye\n")
        sys.exit()

    def help_q(self):
        self.stdout.write("Quit\n")

    def do_lang(self, line):
        lang = line.strip()
        if lang in ['en', 'zh']:
            self.lang = lang
            self.stdout.write("Set lang to {}\n".format(self.lang))
        else:
            self.stdout.write(
                "Current lang {}. \nSet lang by 'lang [en|zh]'\n".format(self.lang))

    def help_lang(self):
        self.stdout.write("Set language. [en|zh]\n")

    def do_c(self, line):
        try:
            params = {
                "session": "{}".format(self.session),
                'Auth': self.key
            }
            r = requests.get(
                '{}/reset_session'.format(self.root_url), params=params)
            ret = r.json().get('ret')
            response = r.json().get('response')
            self.stdout.write(response)
            self.stdout.write('\n')
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def help_c(self):
        self.stdout.write("Clean the memory of the dialog\n")

    def do_rw(self, line):
        try:
            params = {
                "weights": line,
                "Auth": self.key,
                "lang": self.lang,
                "session": self.session
            }
            r = requests.get(
                '{}/set_weights'.format(self.root_url), params=params)
            ret = r.json().get('ret')
            response = r.json().get('response')
            self.stdout.write(response)
            self.stdout.write('\n')
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def help_rw(self):
        s = """
Update the weights of the current chain.
Syntax: rw w1,w2,w3,...
For example, rw .2, .4, .5

"""
        self.stdout.write(s)

    def do_upload(self, line):
        if not os.path.isfile(line):
            self.stdout.write('File "{}" is not found\n'.format(line))
            return
        files = {'zipfile': open(line, 'rb')}
        params = {
            "user": self.user,
            "Auth": self.key,
            "lang": 'en'
        }
        try:
            r = requests.post(
                '{}/upload_character'.format(self.root_url),
                files=files, data=params)
            ret = r.json().get('ret')
            response = r.json().get('response')
            self.stdout.write(response)
            self.stdout.write('\n')
        except Exception:
            self.stdout.write('{}\n'.format(ex))

    def help_upload(self):
        s = """
Upload character package.
Syntax: upload package

"""
        self.stdout.write(s)

    def ping(self):
        try:
            r = requests.get('{}/ping'.format(self.root_url))
            response = r.json().get('response')
            if response == 'pong':
                return True
        except Exception:
            return False

    def do_ping(self, line):
        if self.ping():
            self.stdout.write('pong')
        self.stdout.write('\n')

    def help_ping(self):
        self.stdout.write('Ping the server\n')

    def do_trace(self, line):
        if self.last_response:
            trace = self.last_response.get('trace', None)
            if trace:
                if isinstance(trace, list):
                    trace = ['{}: {}: {}'.format(x, y, z) for x, y, z  in trace]
                    trace = '\n'.join(trace)
                self.stdout.write(trace)
        self.stdout.write('\n')

    do_t = do_trace

    def help_trace(self):
        self.stdout.write('Print the trace of last reponse\n')

    help_t = help_trace

    def _rate(self, rate):
        params = {
            "session": self.session,
            "rate": rate,
            "index": -1,
            "Auth": self.key
        }
        r = requests.get('{}/rate'.format(self.root_url), params=params)
        ret = r.json().get('ret')
        response = r.json().get('response')
        return ret, response

    def do_gd(self, line):
        ret, response = self._rate('good')
        if ret:
            self.stdout.write("[Thanks for rating]\n")
        else:
            self.stdout.write("[Rating failed]\n")

    def help_gd(self):
        self.stdout.write('Rate the last response as GOOD result\n')

    def do_bd(self, line):
        ret, response = self._rate('bad')
        if ret:
            self.stdout.write("[Thanks for rating]\n")
        else:
            self.stdout.write("[Rating failed]\n")

    def help_bd(self):
        self.stdout.write('Rate the last response as BAD result\n')

    def do_dump(self, line):
        params = {
            "session": self.session,
            "Auth": self.key
        }
        r = requests.get(
            '{}/dump_session'.format(self.root_url), params=params)
        if r.status_code == 200:
            fname = '{}.csv'.format(self.session)
            with open(fname, 'w') as f:
                f.write(r.text)
            self.stdout.write('Done\n')
            self.stdout.write('Dump to {}\n'.format(fname))
        else:
            self.stdout.write('Failed, error code {}\n'.format(r.status_code))

    def help_dump(self):
        self.stdout.write('Dump chat history\n')

    do_d = do_dump
    help_d = help_dump

    def do_summary(self, line):
        if line:
            try:
                lookback = int(line)
            except Exception as ex:
                self.stdout.write('{}\n'.format(ex))
                return
        else:
            lookback = 7
        params = {
            "Auth": self.key,
            "lookback": lookback
        }
        r = requests.get('{}/stats'.format(self.root_url), params=params)
        ret = r.json().get('ret')
        response = r.json().get('response')
        if ret:
            self.stdout.write(
                'Customers satisfaction degree {customers_satisfaction_degree:.4f}\n'
                'Number of records {number_of_records}\n'
                'Number of rates {number_of_rates}\n'
                'Number of good rates {number_of_good_rates}\n'
                'Number of bad rates {number_of_bad_rates}\n'.format(**response))
        else:
            self.stdout.write('{}\n'.format(response['err_msg']))

    def help_summary(self):
        self.stdout.write('Report the summary of the chat history\n')
        self.stdout.write('Usage: summary [lookback days]\n')
        self.stdout.write('lookback days: -1 means all\n')

    def process_indicator(self, reply):
        cmd, timeout = None, None
        for match in re.findall(r'\[.*\]', reply):
            match = match.strip()
            match = match.replace(' ', '')
            if match == '[loopback=0]':
                self.cancel_timer()
                return
            match = match.replace(']', '')
            match = match.replace('[', '')
            if '=' in match:
                cmd, timeout = match.split('=')
                self.timeout = float(timeout)/1000
            else:
                cmd = match
            cmd = '[{}]'.format(cmd)
        if self.timeout is not None and cmd is not None:
            self.cancel_timer()
            self.timer = threading.Timer(self.timeout, self.ask, (cmd, ))
            self.timer.start()
            logger.info("Start {} timer with timeout {}".format(
                cmd, self.timeout))

    def do_list_sessions(self, line):
        params = {
            "Auth": self.key
        }
        r = requests.get(
            '{}/sessions'.format(self.root_url), params=params)
        sessions = r.json().get('response')
        if sessions:
            self.stdout.write('sessions: {}\n'.format('\n'.join(sessions)))
        else:
            self.stdout.write('no session\n')

    do_ls = do_list_sessions

    def help_list_sessions(self):
        self.stdout.write('List the current sessions\n')

    help_ls = help_list_sessions

    def do_ns(self, line):
        self.start_session(True)

    def help_ns(self):
        self.stdout.write('Start new session\n')
