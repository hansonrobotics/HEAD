import os
from chatbot.aiml import Kernel
import logging
import re
from config import CHARACTER_PATH
from chatbot.utils import shorten
from collections import defaultdict
from pprint import pformat

TYPE_AIML='aiml'
TYPE_CS='cs'
TYPE_DEFAULT='default'

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
        self.type = TYPE_DEFAULT

    def get_properties(self):
        return self.properties

    def set_properties(self, props):
        self.properties.update(props)

    def respond(self, question, lang, session=None, query=False, request_id=None):
        raise NotImplementedError

    def refresh(self, session):
        raise NotImplementedError

    def get_context(self, session):
        return {}

    def set_context(self, session, context):
        raise NotImplementedError

    def remove_context(self, session, key):
        raise NotImplementedError

    def is_command(self, question):
        return False

    def is_favorite(self, question):
        return False

    def __repr__(self):
        return "<Character id: {}, name: {}, level: {}>".format(
            self.id, self.name, self.level)


class DefaultCharacter(Character):

    def set_context(self, session, context):
        session.sdata.set_context(self.id, context)

    def get_context(self, session):
        return session.sdata.get_context(self.id)

    def refresh(self, session):
        session.sdata.reset_context(self.id)

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
        self.response_limit = 512
        self.type = TYPE_AIML

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

    def respond(self, question, lang, session, query, request_id=None):
        ret = {}
        ret['text'] = ''
        ret['botid'] = self.id
        ret['botname'] = self.name
        ret['repeat'] = ''

        sid = session.sid
        answer, res = '', ''
        if lang not in self.languages:
            return ret
        elif re.search(r'\[.*\]', question):
            return ret

        chat_tries = 0
        answer = self.kernel.respond(question, sid, query)
        answer, res = shorten(answer, self.response_limit)

        if self.non_repeat:
            while chat_tries < self.max_chat_tries:
                if answer and session.check(question, answer):
                    break
                answer = self.kernel.respond(question, sid, query)
                answer, res = shorten(answer, self.response_limit)
                chat_tries += 1
        if answer:
            if not session.check(question, answer):
                ret['repeat'] = answer
                answer = ''
                self.logger.warn("Repeat answer")
        if res:
            self.kernel.setPredicate('continue', res, sid)
            self.logger.info("Set predicate continue={}".format(res))
        ret['text'] = answer
        ret['emotion'] = self.kernel.getPredicate('emotion', sid)
        ret['performance'] = self.kernel.getPredicate('performance', sid)
        traces = self.kernel.getTraceDocs()
        if traces:
            self.logger.debug("Trace: {}".format(traces))
            patterns = []
            for trace in traces:
                match_obj = self.trace_pattern.match(trace)
                if match_obj:
                    patterns.append(match_obj.group('pname'))
            ret['pattern'] = patterns
            if patterns:
                first = patterns[0]
                if '*' in first or '_' in first:
                    pattern_len = len(first.strip().split())
                    if '*' not in first:
                        ret['ok_match'] = True
                    if pattern_len>3 and pattern_len>0.9*len(question.strip().split()):
                        ret['ok_match'] = True
                else:
                    ret['exact_match'] = True
            traces = replace_aiml_abs_path(traces)
            ret['trace'] = '\n'.join(traces)
        return ret

    def refresh(self, session):
        sid = session.sid
        self.kernel._deleteSession(sid)
        self.logger.info("Character is refreshed")

    def get_context(self, session):
        sid = session.sid
        return self.kernel.getSessionData(sid)

    def set_context(self, session, context):
        assert isinstance(context, dict)
        sid = session.sid
        for k, v in context.iteritems():
            if k.startswith('_'):
                continue
            self.kernel.setPredicate(k, v, sid)
            self.logger.info("Set predicate {}={}".format(k, v))
            if k in ['firstname', 'fullname']:
                self.kernel.setPredicate('name', v, sid)

    def remove_context(self, session, key):
        sid = session.sid
        if key in self.get_context(session).keys():
            del self.kernel._sessions[sid][key]
            self.logger.info("Removed context {}".format(key))
            return True
        else:
            self.logger.debug("No such context {}".format(key))
            return False

    def get_templates(self):
        templates = []
        root = self.kernel._brain._root
        self.kernel._brain.get_templates(root, templates)
        return templates

    def print_duplicated_patterns(self):
        patterns = defaultdict(list)
        for t in self.get_templates():
            key = (t[1]['pattern'].lower(),
                t[1]['that'].lower(),
                t[1]['topic'].lower())
            patterns[key].append(t[1])
        for pattern in patterns:
            if len(patterns[pattern]) > 1:
                self.logger.error("Duplicated patterns {}\n{}\n".format(
                    len(patterns[pattern]), pformat(patterns[pattern])))

    def said(self, session, text):
        sid = session.sid
        outputHistory = self.kernel.getPredicate(self.kernel._outputHistory, sid)
        if isinstance(outputHistory, list):
            outputHistory.append(text)
            self.logger.info("Add '{}' to output history".format(text))
