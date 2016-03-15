import sys
import aiml

class Character(object):
    def __init__(self, name):
        self.name = name
        self.character = aiml.Kernel()
        self.aiml_files = []
        self.character.verbose(False)
        self.properties = {}

    def load_aiml_files(self, aiml_files):
        for f in aiml_files:
            self.character.learn(f)
            if f not in self.aiml_files:
                self.aiml_files.append(f)

    def reload(self):
        self.load_aiml_files(self.aiml_files)

    def set_property_file(self, propname):
        try:
            with open(propname) as f:
                for line in f:
                    parts = line.split('=')
                    key = parts[0].strip()
                    value = parts[1].strip()
                    self.character.setBotPredicate(key, value)
                    self.properties[key] = value
        except Exception:
            print >>sys.stderr, "Couldn't open {}".format(propname)

    def set_properties(self, props):
        for key, value in props.iteritems():
            self.character.setBotPredicate(key, value)
            self.properties[key] = value

    def get_properties(self):
        return self.properties

    def respond(self, question, session=None):
        ret = {}
        ret['response'] = self.character.respond(question, session)
        ret['emotion'] = self.character.getPredicate('emotion', session)
        ret['botname'] = self.name
        return ret

    def __repr__(self):
        return "<Character name {}>".format(self.name)

