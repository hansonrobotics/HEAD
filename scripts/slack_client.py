#!/usr/bin/env python

import os
import time
import logging
import re
import sys

from slackclient import SlackClient
CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, '../src'))
from chatbot.client import Client
from chatbot.server.session import SessionManager

HR_CHATBOT_AUTHKEY = os.environ.get('HR_CHATBOT_AUTHKEY', 'AAAAB3NzaC')
SLACKBOT_API_TOKEN = os.environ.get('SLACKBOT_API_TOKEN')
SLACKTEST_TOKEN = os.environ.get('SLACKTEST_TOKEN')

logger = logging.getLogger('hr.chatbot.slackclient')

def format_trace(traces):
    pattern = re.compile(
        r'../(?P<fname>.*), (?P<tloc>\(.*\)), (?P<pname>.*), (?P<ploc>\(.*\))')
    line_pattern = re.compile(r'\(line (?P<line>\d+), column \d+\)')
    urlprefix = "https://github.com/hansonrobotics/character_dev/blob/update"
    formated_traces = []
    for trace in traces:
        matchobj = pattern.match(trace)
        if matchobj:
            fname = matchobj.groupdict()['fname']
            tloc = matchobj.groupdict()['tloc']
            pname = matchobj.groupdict()['pname']
            ploc = matchobj.groupdict()['ploc']
            tline = line_pattern.match(tloc).group('line')
            pline = line_pattern.match(ploc).group('line')

            p = '<{urlprefix}/{fname}#L{pline}|{pname} {ploc}>'.format(
                pname=pname, urlprefix=urlprefix, fname=fname, pline=pline, ploc=ploc)
            t = '<{urlprefix}/{fname}#L{tline}|{tloc}>'.format(
                urlprefix=urlprefix, fname=fname, tline=tline, tloc=tloc)
            formated_trace = '{p}, {t}, {fname}'.format(fname=fname, p=p, t=t)
            formated_traces.append(formated_trace)
    return formated_traces


class HRSlackBot(object):

    def __init__(self, host, port, botname):
        self.sc = SlackClient(SLACKBOT_API_TOKEN)
        self.sc.rtm_connect()
        self.botname = botname
        self.host = host
        self.port = str(port)
        self.lang = 'en'
        self.icon_url = 'https://avatars.slack-edge.com/2016-05-30/46725216032_4983112db797f420c0b5_48.jpg'
        self.session_manager = SessionManager()

    def send_message(self, channel, attachments):
        self.sc.api_call(
            "chat.postMessage", channel=channel,
            attachments=attachments, username=self.botname.title(),
            icon_url=self.icon_url)

    def run(self):
        while True:
            time.sleep(0.2)
            messages = self.sc.rtm_read()
            if not messages:
                continue
            for message in messages:
                if message['type'] != u'message':
                    continue
                if message.get('subtype') == u'bot_message':
                    continue
                usr_obj = self.sc.api_call(
                    'users.info', token=SLACKTEST_TOKEN, user=message['user'])
                if not usr_obj['ok']:
                    continue
                profile = usr_obj['user']['profile']
                name = profile.get('first_name') or profile.get('email')
                question = message.get('text')
                channel = message.get('channel')

                sid = self.session_manager.get_sid(name, self.botname)
                if sid is not None:
                    session = self.session_manager.get_session(sid)
                    assert session is not None and hasattr(session.sdata, 'client')
                    client = session.sdata.client
                else:
                    client = Client(HR_CHATBOT_AUTHKEY, username=name,
                        botname=self.botname, host=self.host, port=self.port,
                        response_listener=self)
                    self.session_manager.add_session(name, self.botname, client.session)
                    session = self.session_manager.get_session(client.session)
                    session.sdata.client = client
                    session.sdata.channel = channel

                logger.info("Question {}".format(question))
                if question in [':+1:', ':slightly_smiling_face:', ':)', 'gd']:
                    ret, _ = client._rate('good')
                    if ret:
                        logger.info("Rate good")
                        answer = 'Thanks for rating'
                        color = 'good'
                    else:
                        logger.info("Rate failed")
                        answer = 'Rating failed'
                        color = 'danger'
                    attachments = [{
                        'title': answer,
                        'color': color,
                        'fallback': answer
                    }]
                    self.send_message(channel, attachments)
                    continue
                if question in [':-1:', ':disappointed:', ':(', 'bd']:
                    ret, _ = client._rate('bad')
                    if ret:
                        logger.info("Rate bad")
                        answer = 'Thanks for rating'
                        color = 'good'
                    else:
                        logger.info("Rate failed")
                        answer = 'Rating failed'
                        color = 'danger'
                    attachments = [{
                        'title': answer,
                        'color': color,
                        'fallback': answer
                    }]
                    self.send_message(channel, attachments)
                    continue

                client.ask(question)

    def on_response(self, sid, response):
        answer = ''
        title = ''
        session = self.session_manager.get_session(sid)
        if session is None:
            return
        channel = session.sdata.channel
        if response is None or not response.get('text'):
            answer = u"Sorry, I can't answer it right now"
        else:
            answer = response.get('text')
            trace = response.get('trace', '')
            botid = response.get('botid', '')
            formated_trace = format_trace(trace)
            if formated_trace:
                title = 'answered by {}\ntrace:\n{}'.format(
                    botid, '\n'.join(formated_trace))
        attachments = [{
            'pretext': answer,
            'title': title,
            'color': '#3AA3E3',
            'fallback': answer,
        }]
        self.send_message(channel, attachments)

if __name__ == '__main__':
    logging.basicConfig()
    logging.getLogger().setLevel(logging.INFO)
    logging.getLogger('urllib3').setLevel(logging.WARN)
    host = 'localhost'
    port = 8001
    if len(sys.argv) < 2:
        print "Usage:", sys.argv[0], "botname"
        sys.exit(1)
    botname = sys.argv[1]
    while True:
        try:
            HRSlackBot(host, port, botname).run()
        except Exception as ex:
            logger.error(ex)
