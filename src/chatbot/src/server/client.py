import cmd
import requests
import json
import os
import uuid

import sys
reload(sys)
sys.setdefaultencoding('utf-8')
VERSION = 'v1'

key='AAAAB3NzaC'

class Client(cmd.Cmd, object):
    def __init__(self):
        super(Client, self).__init__()
        self.prompt = '[me]: '
        self.botid = 'sophia'
        self.chatbot_ip = 'localhost'
        self.chatbot_port = '8001'
        self.chatbot_url = 'http://{}:{}/{}'.format(
            self.chatbot_ip, self.chatbot_port, VERSION)
        self.lang = 'en'
        self.session = uuid.uuid1()

    def ask(self, question):
        params = {
            "botid": "{}".format(self.botid),
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
            self.stdout.write("QA error: error code {}, botid {}, question {}, lang {}\n".format(
                ret, self.botid, question, self.lang))

        response = {'text': '', 'emotion': '', 'botid': '', 'botname': ''}
        response.update(r.json().get('response'))

        return response

    def list_chatbot(self, botid=None):
        r = requests.get(
            '{}/chatbots'.format(self.chatbot_url),
            params={'Auth':key, 'botid':botid})
        chatbots = r.json().get('response')
        return chatbots

    def default(self, line):
        try:
            if line:
                response = self.ask(line)
                self.stdout.write('{}[by {}]: {}\n'.format(
                    self.botid, response.get('botid'),
                    response.get('text')))
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def do_list(self, line):
        chatbots = []
        try:
            if line == 'all':
                chatbots = self.list_chatbot(line)
            else:
                chatbots = self.list_chatbot(self.botid)
            chatbots = ['{}: {}'.format(c,w) if c!=self.botid else \
                '[{}]: {}'.format(c, w) for c, w in chatbots]
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
            chatbots = self.list_chatbot()
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))
            return
        characters = [c[0] for c in self.list_chatbot()]
        if line in characters:
            self.botid = line
            self.stdout.write("Select chatbot {}\n".format(self.botid))
        else:
            self.stdout.write("No such chatbot {}\n".format(line))

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

    def _update(self):
        params = {
            "botid": "{}".format(self.botid),
            'Auth': key
        }
        r = requests.get('{}/update'.format(self.chatbot_url), params=params)
        return r.json().get('ret')

    def do_update(self, line):
        try:
            params = {
                "botid": "{}".format(self.botid),
                'Auth': key
            }
            r = requests.get('{}/update'.format(self.chatbot_url), params=params)
            ret = r.json().get('ret')
            response = r.json().get('response')
            self.stdout.write(response)
            self.stdout.write('\n')
            if ret:
                self.stdout.write("Update successfully\n")
            else:
                self.stdout.write("Update failed\n")
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def help_update(self):
        self.stdout.write("Update current chatbot\n")

    def do_load_sheet_keys(self, line):
        try:
            params = {
                "botid":"{}".format(self.botid),
                "sheet_keys":line.strip(),
                'Auth':key
            }
            r = requests.get('{}/set_keys'.format(self.chatbot_url), params=params)
            ret = r.json().get('ret')
            response = r.json().get('response')
            self.stdout.write(response)
            self.stdout.write('\n')
            if ret:
                self.stdout.write("Load sheet keys successfully\n")
            else:
                self.stdout.write("Load sheet keys failed\n")
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def help_load_sheet_keys(self):
        self.stdout.write("Load sheet keys to the current chatbot\n")

    def do_commit(self, line):
        try:
            params = {
                "botid":"{}".format(self.botid),
                'Auth':key
            }
            r = requests.get('{}/commit'.format(self.chatbot_url), params=params)
            ret = r.json().get('ret')
            response = r.json().get('response')
            self.stdout.write(response)
            self.stdout.write('\n')
            if ret:
                self.stdout.write("Commit sheet successfully\n")
            else:
                self.stdout.write("Commit sheet failed\n")
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def help_commit(self):
        self.stdout.write("Commit the current sheet to GitHub\n")

    def do_lang(self, line):
        lang = line.strip()
        if lang in ['en', 'zh']:
            self.lang = lang
            self.stdout.write("Set lang to {}\n".format(self.lang))
        else:
            self.stdout.write("Invalid argument. lang [en|zh]\n")

    def help_lang(self):
        self.stdout.write("Set language. [en|zh]\n")

    def do_c(self, line):
        try:
            params = {
                "session":"{}".format(self.session),
                'Auth':key
            }
            r = requests.get(
                '{}/clean_cache'.format(self.chatbot_url), params=params)
            ret = r.json().get('ret')
            response = r.json().get('response')
            self.stdout.write(response)
            self.stdout.write('\n')
            if ret:
                self.stdout.write("Memory cleaned\n")
            else:
                self.stdout.write("Memory clean failed\n")
        except Exception as ex:
            self.stdout.write('{}\n'.format(ex))

    def help_c(self):
        self.stdout.write("Clean the memory of the dialog\n")

    def do_rw(self, line):
        try:
            params = {
                "botid": "{}".format(self.botid),
                "weights": line,
                "Auth": key
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

if __name__ == '__main__':
    client = Client()
    client.cmdloop()

