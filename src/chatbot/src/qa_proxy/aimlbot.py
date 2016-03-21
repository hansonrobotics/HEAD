
import os
import logging
import chatbot
import aiml
from qa_proxy.QA import ProxyQA, LanguageCondition

logger = logging.getLogger('hr.chatbot.aimlbot')

class AIMLBot(ProxyQA):

    def __init__(self):
        super(AIMLBot, self).__init__()
        self._kernel = aiml.Kernel()
        self.add_condition(LanguageCondition('en'))

    def add_aimls(self, aiml_dir):
        self._kernel.learn(os.sep.join([aiml_dir, '*.aiml']))

    def add_aiml(self, aiml_file):
        self._kernel.learn(aiml_file)

    def load_properties(self, prop_file):
        if os.path.isfile(prop_file):
            with open(prop_file) as f:
                for line in f:
                    parts = line.split('=')
                    key = parts[0].strip()
                    value = parts[1].strip()
                    self._kernel.setBotPredicate(key, value)
        else:
            logger.warn("Property file {} doesn't exist".format(prop_file))

    def _ask(self, question, *args, **kwargs):
        return self._kernel.respond(question)

aimlbot = AIMLBot()

if __name__ == '__main__':
    cwd = os.path.dirname(os.path.realpath(__file__))
    logging.basicConfig()
    aimlbot.add_aimls(os.path.join(cwd, '../../aiml/'))
    aimlbot.load_properties(os.path.join(cwd, '../../aiml/bot.properties'))
    print aimlbot.ask('What is your name', lang='en')
