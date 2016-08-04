import cmd
import requests
import json
import os
import time

import sys
reload(sys)
sys.setdefaultencoding('utf-8')
VERSION = 'v1.1'

class Client(cmd.Cmd, object):
    def __init__(self, username, key, *args, **kwargs):
        super(Client, self).__init__(*args, **kwargs)
        self.user = username
        self.key = key
        self.prompt = '[me]: '
        self.botname = 'sophia'
        self.chatbot_ip = 'localhost'
        self.chatbot_port = '8001'
        self.chatbot_url = 'http://{}:{}/{}'.format(
            self.chatbot_ip, self.chatbot_port, VERSION)
        self.lang = 'en'
        self.session = None
        self.last_response = None
        self.do_conn()

    def set_sid(self):
        params = {
            "Auth": self.key,
            "botname": self.botname,
            "user": self.user
        }
        r = None
        retry = 3
        while r is None and retry > 0:
            try:
                r = requests.get('{}/start_session'.format(self.chatbot_url), params=params)
            except Exception:
                retry -= 1
                self.stdout.write('.')
                self.stdout.flush()
                time.sleep(1)
        if r is None:
            self.stdout.write("Can't get session\nPlease check the url {}\n".format(self.chatbot_url))
            return
        ret = r.json().get('ret')
        if r.status_code != 200:
            self.stdout.write("Request error: {}\n".format(r.status_code))
        self.session = r.json().get('sid')
        self.stdout.write("Init session {}\n".format(self.session))

    def ask(self, question):
        params = {
            "question": "{}".format(question),
            "session": self.session,
            "lang": self.lang,
            "Auth": self.key
        }
        r = requests.get('{}/chat'.format(self.chatbot_url), params=params)
        ret = r.json().get('ret')
        if r.status_code != 200:
            self.stdout.write("Request error: {}\n".format(r.status_code))

        if ret != 0:
            self.stdout.write("QA error: error code {}, botname {}, question {}, lang {}\n".format(
                ret, self.botname, question, self.lang))

        response = {'text': '', 'emotion': '', 'botid': '', 'botname': ''}
        response.update(r.json().get('response'))

        return ret, response

    def list_chatbot(self):
        params={'Auth':self.key, 'lang':self.lang, 'session': self.session}
        r = requests.get(
            '{}/chatbots'.format(self.chatbot_url), params=params)
        chatbots = r.json().get('response')
        return chatbots

    def list_chatbot_names(self):
        params={'Auth':self.key, 'lang':self.lang, 'session': self.session}
        r = requests.get(
            '{}/bot_names'.format(self.chatbot_url), params=params)
        names = r.json().get('response')
        return names

    def default(self, line):
        try:
            if line:
                ret, response = self.ask(line)
                if ret != 0:
                    self.do_conn()
                    ret, response = self.ask(line)
                else:
                    self.last_response = response
                    self.stdout.write('{}[by {}]: {}\n'.format(
                        self.botname, response.get('botid'),
                        response.get('text')))
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def do_list(self, line):
        chatbots = []
        try:
            chatbots = self.list_chatbot()
            chatbots = ['{}: weight: {} level: {}'.format(c,w,l) for c, w, l in chatbots]
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
                self.set_sid()
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
                self.chatbot_url = 'http://{}:{}/{}'.format(
                    self.chatbot_ip, self.chatbot_port, VERSION)
            except Exception:
                self.stdout.write("Wrong conn argument\n")
                self.help_conn()
                return
        self.stdout.write("Connecting.")
        self.set_sid()

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
        self.chatbot_url = 'http://{}:{}/{}'.format(
            self.chatbot_ip, self.chatbot_port, VERSION)


    def help_ip(self):
        s = """
Set the IP address of chatbot server
Syntax: ip xxx.xxx.xxx.xxx
For example, ip 127.0.0.1

"""
        self.stdout.write(s)

    def do_port(self, line):
        self.chatbot_port = line
        self.chatbot_url = 'http://{}:{}/{}'.format(
            self.chatbot_ip, self.chatbot_port, VERSION)

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
            self.stdout.write("Current lang {}. \nSet lang by 'lang [en|zh]'\n".format(self.lang))

    def help_lang(self):
        self.stdout.write("Set language. [en|zh]\n")

    def do_c(self, line):
        try:
            params = {
                "session":"{}".format(self.session),
                'Auth':self.key
            }
            r = requests.get(
                '{}/reset_session'.format(self.chatbot_url), params=params)
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
                '{}/set_weights'.format(self.chatbot_url), params=params)
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
                '{}/upload_character'.format(self.chatbot_url),
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
            r = requests.get('{}/ping'.format(self.chatbot_url))
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
            self.stdout.write('\n'.join(trace))
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
        r = requests.get('{}/rate'.format(self.chatbot_url), params=params)
        ret = r.json().get('ret')
        response = r.json().get('response')
        if ret:
            self.stdout.write("[Thanks for rating]\n")
        else:
            self.stdout.write("[Rating failed]\n")

    def do_gd(self, line):
        ret, response = self._rate('good')

    def help_gd(self):
        self.stdout.write('Rate the last response as GOOD result\n')

    def do_bd(self, line):
        self._rate('bad')

    def help_bd(self):
        self.stdout.write('Rate the last response as BAD result\n')

    def do_dump(self, line):
        params = {
            "session": self.session,
            "Auth": self.key
        }
        r = requests.get('{}/dump_session'.format(self.chatbot_url), params=params)
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

