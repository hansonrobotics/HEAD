import os
from chatbot.aiml import Kernel
import logging


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
        sid = session.sid
        answer = ''
        if lang not in self.languages:
            answer = ''
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
        ret['text'] = answer
        ret['emotion'] = self.kernel.getPredicate('emotion', sid)
        ret['botid'] = self.id
        ret['botname'] = self.name
        trace = self.kernel.getTraceDocs()
        if trace:
            self.logger.info("Trace: {}".format(trace))
            ret['trace'] = trace
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
