import os
from chatbot.aiml import Kernel
import logging
import re
from config import CHARACTER_PATH
from chatbot.utils import shorten


class Character(object):

    def __init__(self, id, name, level=99, dynamic_level=False):
        self.id = id
        self.name = name
        self.level = level
        self.dynamic_level = dynamic_level
        self.properties = {}
        self.weight = 1  # How likely its response is used. [0-1]
        self.languages = ['en']  # List of lanugages it supports
        self.local = True
        self.non_repeat = True
        self.logger = logging.getLogger('hr.chatbot.character.{}'.format(id))

    def get_properties(self):
        return self.properties

    def set_properties(self, props):
        self.properties.update(props)

    def respond(self, question, lang, session=None, query=False):
        raise NotImplementedError

    def refresh(self, sid):
        raise NotImplementedError

    def get_context(self, sid):
        return {}

    def set_context(self, sid, context):
        raise NotImplementedError

    def is_command(self, question):
        return False

    def __repr__(self):
        return "<Character id: {}, name: {}, level: {}>".format(
            self.id, self.name, self.level)


def replace_aiml_abs_path(trace):
    if isinstance(trace, list):
        for path in CHARACTER_PATH.split(','):
            path = path.strip()
            if not path:
                continue
            path = path + '/'
            trace = [f.replace(path, '') for f in trace]
    return trace


class AIMLCharacter(Character):

    def __init__(self, id, name, level=99):
        super(AIMLCharacter, self).__init__(id, name, level)
        self.kernel = Kernel()
        self.aiml_files = []
        self.kernel.verbose(True)
        self.current_topic = ''
        self.counter = 0
        self.N = 10  # How many times of reponse on the same topic
        self.languages = ['en']
        self.max_chat_tries = 5
        self.trace_pattern = re.compile(
            r'.*/(?P<fname>.*), (?P<tloc>\(.*\)), (?P<pname>.*), (?P<ploc>\(.*\))')
        self.response_limit = 300

    def load_aiml_files(self, kernel, aiml_files):
        errors = []
        for f in aiml_files:
            if '*' not in f and not os.path.isfile(f):
                self.logger.warn("{} is not found".format(f))
            errors.extend(kernel.learn(f))
            self.logger.info("Load {}".format(f))
            if f not in self.aiml_files:
                self.aiml_files.append(f)
        return errors

    def set_property_file(self, propname):
        try:
            with open(propname) as f:
                for line in f:
                    parts = line.split('=')
                    key = parts[0].strip()
                    value = parts[1].strip()
                    self.kernel.setBotPredicate(key, value)
                    self.properties[key] = value
                self.logger.info("Set properties file {}".format(propname))
        except Exception:
            self.logger.error("Couldn't open {}".format(propname))

    def set_properties(self, props):
        super(AIMLCharacter, self).set_properties(props)
        for key, value in self.properties.iteritems():
            self.kernel.setBotPredicate(key, value)

    def check_reset_topic(self, session):
        """If it's on the same topic for N round, then reset the topic"""
        topic = self.kernel.getPredicate('topic', session).strip()
        if not topic:
            self.reset_topic(session)
            return
        if topic == self.current_topic:
            self.counter += 1
            self.logger.info('Topic is continued {} {}'.format(
                topic, self.counter))
        else:
            self.counter = 0
            self.logger.info('Topic is changed from "{}" to "{}", '
                        'reset counter'.format(self.current_topic, topic))
            self.current_topic = topic
        if self.counter >= self.N:
            self.counter = 0
            self.reset_topic(session)

    def reset_topic(self, session):
        self.current_topic = ''
        self.kernel.setPredicate('topic', '', session)
        self.logger.info("Topic is reset")

    def respond(self, question, lang, session, query):
        ret = {}
        ret['text'] = ''
        ret['botid'] = self.id
        ret['botname'] = self.name

        sid = session.sid
        answer = ''
        if lang not in self.languages:
            return ret
        elif re.search(r'\[.*\]', question):
            return ret
        else:
            chat_tries = 0
            answer = self.kernel.respond(question, sid, query)
            if self.non_repeat:
                while chat_tries < self.max_chat_tries:
                    if session.check(question, answer):
                        break
                    answer = self.kernel.respond(question, sid, query)
                    chat_tries += 1
                if not session.check(question, answer):
                    answer = ''
                    ret['repeat'] = True
                    self.logger.warn("Repeat answer")
        answer, res = shorten(answer, self.response_limit)
        if res:
            self.kernel.setPredicate('tellmore', res)
            self.logger.info("Set predicate tellmore={}".format(res))
        ret['text'] = answer
        ret['emotion'] = self.kernel.getPredicate('emotion', sid)
        traces = self.kernel.getTraceDocs()
        if traces:
            self.logger.info("Trace: {}".format(traces))
            patterns = []
            for trace in traces:
                match_obj = self.trace_pattern.match(trace)
                if match_obj:
                    patterns.append(match_obj.group('pname'))
            ret['pattern'] = patterns
            if patterns:
                first = patterns[0]
                if '*' in first or '_' in first:
                    if len(first.strip().split())>0.9*len(question.strip().split()):
                        ret['ok_match'] = True
                else:
                    ret['exact_match'] = True
            traces = replace_aiml_abs_path(traces)
            ret['trace'] = '\n'.join(traces)
        return ret

    def refresh(self, sid):
        self.kernel._deleteSession(sid)
        self.logger.info("Character is refreshed")

    def get_context(self, sid):
        return self.kernel.getSessionData(sid)

    def set_context(self, sid, context):
        assert isinstance(context, dict)
        for k, v in context.iteritems():
            if k.startswith('_'):
                continue
            self.kernel.setPredicate(k, v, sid)
            self.logger.info("Set predicate {}={}".format(k, v))
