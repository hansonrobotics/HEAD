import cmd
import requests
import json
import os

import sys
reload(sys)
sys.setdefaultencoding('utf-8')
VERSION = 'v1.1'

key='AAAAB3NzaC'

class Client(cmd.Cmd, object):
    def __init__(self):
        super(Client, self).__init__()
        self.prompt = '[me]: '
        self.botname = 'sophia'
        self.chatbot_ip = 'localhost'
        self.chatbot_port = '8001'
        self.chatbot_url = 'http://{}:{}/{}'.format(
            self.chatbot_ip, self.chatbot_port, VERSION)
        self.lang = 'en'
        self.user = 'client'
        self.session = None
        self.set_sid()

    def set_sid(self):
        params = {
            "Auth": key,
            "botname": self.botname,
            "user": self.user
        }
        r = requests.get('{}/start_session'.format(self.chatbot_url), params=params)
        ret = r.json().get('ret')
        if r.status_code != 200:
            self.stdout.write("Request error: {}\n".format(r.status_code))
        self.session = r.json().get('sid')
        self.stdout.write("New session {}\n".format(self.session))

    def ask(self, question):
        params = {
            "question": "{}".format(question),
            "session": self.session,
            "lang": self.lang,
            "Auth": key
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
        params={'Auth':key, 'lang':self.lang, 'session': self.session}
        r = requests.get(
            '{}/chatbots'.format(self.chatbot_url), params=params)
        chatbots = r.json().get('response')
        return chatbots

    def list_chatbot_names(self):
        params={'Auth':key, 'lang':self.lang, 'session': self.session}
        r = requests.get(
            '{}/bot_names'.format(self.chatbot_url), params=params)
        names = r.json().get('response')
        return names

    def default(self, line):
        try:
            if line:
                ret, response = self.ask(line)
                if ret != 0:
                    self.set_sid()
                    ret, response = self.ask(line)
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

    def do_conn(self, line):
        self.chatbot_url = line

    def help_conn(self):
        s = """
Connect to chatbot server
Syntax: conn url:port
For example, conn 127.0.0.1:8001

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
                'Auth':key
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
                "Auth": key,
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
            "Auth": key,
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


if __name__ == '__main__':
    client = Client()
    client.cmdloop()

