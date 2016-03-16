import cmd
import requests
import json
import sys

class Client(cmd.Cmd, object):
    def __init__(self):
        super(Client, self).__init__()
        self.prompt = '[me]: '
        self.botname = 'futurist_sophia'
        self.chatbot_url = 'http://localhost:8001'

    def ask(self, question):
        r = requests.post(self.chatbot_url,
                data = json.dumps(
                    {"botname":"{}".format(self.botname),
                    "question":"{}".format(question),
                    "session":"0"}),
                headers = {"Content-Type": "application/json"}
        )
        ret = r.json().get('ret')
        if r.status_code != 200:
            self.stdout.write("Request error: {}".format(r.status_code))

        if ret != 0:
            self.stdout.write("QA error: error code {}, botname {}, question {}".format(
                ret, self.botname, question))

        response = r.json().get('response', {})

        if r.status_code != 200 or ret != 0 or not response:
            response['response'] = question
            response['botname'] = 'mimic_bot'

        return response


    def default(self, line):
        try:
            if line:
                response = self.ask(line)
            self.stdout.write('{}[by {}]: {}\n'.format(
                self.botname, response.get('botname'), response.get('response')))
        except Exception as ex:
            print ex

    def do_list(self, line):
        s = """
Current
    {}

Chatbot
    sophia
    han
    pkd
    futurist_sophia

""".format(self.botname)
        self.stdout.write(s)

    def help_list(self):
        self.stdout.write("List chatbot names")

    def do_chatbot(self, line):
        self.botname = line
        self.stdout.write("Set chatbot to {}".format(self.botname))

    def help_chatbot(self):
        self.stdout.write("Set chatbot name")

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
        self.stdout.write("Bye")
        sys.exit()

    def help_q(self):
        self.stdout.write("Quit")

if __name__ == '__main__':
    client = Client()
    client.cmdloop()

