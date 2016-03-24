import cmd
import requests
import json
import os

import sys
reload(sys)
sys.setdefaultencoding('utf-8')

class Client(cmd.Cmd, object):
    def __init__(self):
        super(Client, self).__init__()
        self.prompt = '[me]: '
        self.botid = 'han'
        self.chatbot_url = 'http://localhost:8001'

    def ask(self, question):
        r = requests.post(self.chatbot_url,
                data = json.dumps(
                    {"botid":"{}".format(self.botid),
                    "question":"{}".format(question),
                    "session":"0"}),
                headers = {"Content-Type": "application/json",
                           "Auth": 'AAAAB3NzaC'}
        )
        ret = r.json().get('ret')
        if r.status_code != 200:
            self.stdout.write("Request error: {}\n".format(r.status_code))

        if ret != 0:
            self.stdout.write("QA error: error code {}, botid {}, question {}\n".format(
                ret, self.botid, question))

        response = {'text': '', 'emotion': '', 'botid': '', 'botname': ''}
        response.update(r.json().get('response'))

        return response

    def list_chatbot(self):
        r = requests.get(os.path.join(self.chatbot_url, 'chatbots'))
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
            chatbots = self.list_chatbot()
            chatbots = [c if c!=self.botid else '[{}]'.format(c) for c in chatbots]
            self.stdout.write('\n'.join(chatbots))
            self.stdout.write('\n')
        except requests.exceptions.ConnectionError as ex:
            self.stdout.write('{}\n'.format(ex))

    def help_list(self):
        self.stdout.write("List chatbot names\n")

    def do_chatbot(self, line):
        try:
            chatbots = self.list_chatbot()
        except requests.exceptions.ConnectionError as ex:
            self.stdout.write('{}\n'.format(ex))
            return
        if line in self.list_chatbot():
            self.botid = line
            self.stdout.write("Set chatbot to {}\n".format(self.botid))
        else:
            self.stdout.write("No such chatbot {}\n".format(line))

    def help_chatbot(self):
        self.stdout.write("Set chatbot name\n")

    def do_conn(self, line):
        self.chatbot_url = line

    def help_conn(self):
        s = """

Connect to chatbot server
Syntax: conn url:port
For example, conn 127.0.0.1:8001

"""
        self.stdout.write(s)

    def do_q(self, line):
        self.stdout.write("Bye\n")
        sys.exit()

    def help_q(self):
        self.stdout.write("Quit\n")


if __name__ == '__main__':
    client = Client()
    client.cmdloop()

