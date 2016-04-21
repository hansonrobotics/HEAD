import os
import sys
import aiml
import logging
from zipfile import ZipFile
import datetime as dt
import shutil
from gsheet_chatter import getWorkSheets, batch_csv2aiml
import subprocess

logger = logging.getLogger('hr.chatbot.character')
CWD = os.path.dirname(os.path.realpath(__file__))
GIT_DIR = os.path.expanduser('~/.hr/aiml/character_dev')
WORKING_BRANCH = 'auto_commit_only'
os.system('cd {} && git checkout {}'.format(GIT_DIR, WORKING_BRANCH))
COMMENT = 'Update character'

class Character(object):
    def __init__(self, id, name, level=99):
        self.id = id
        self.name = name
        self.level = level
        self.properties = {}

    def get_properties(self):
        return self.properties

    def set_properties(self, props):
        self.properties.update(props)

    def respond(self, question, session=None):
        raise NotImplementedError

    def __repr__(self):
        return "<Character id: {}, name: {}, level: {}>".format(
            self.id, self.name, self.level)

class AIMLCharacter(Character):
    def __init__(self, id, name, level=99):
        super(AIMLCharacter, self).__init__(id, name, level)
        self.kernel = aiml.Kernel()
        self.aiml_files = []
        self.kernel.verbose(False)

    def load_aiml_files(self, kernel, aiml_files):
        for f in aiml_files:
            errors = kernel.learn(f)
            if errors:
                raise Exception("Load {} error\n{}".format(
                    os.path.basename(f), errors[0][1]))
            logger.info("[{}] Load {}".format(self.id, f))
            if f not in self.aiml_files:
                self.aiml_files.append(f)

    def set_property_file(self, propname):
        try:
            with open(propname) as f:
                for line in f:
                    parts = line.split('=')
                    key = parts[0].strip()
                    value = parts[1].strip()
                    self.kernel.setBotPredicate(key, value)
                    self.properties[key] = value
        except Exception:
            logger.error("Couldn't open {}".format(propname))

    def set_properties(self, props):
        super(AIMLCharacter, self).set_properties(props)
        for key, value in self.properties.iteritems():
            self.kernel.setBotPredicate(key, value)

    def respond(self, question, session=None):
        ret = {}
        ret['text'] = self.kernel.respond(question, session)
        ret['emotion'] = self.kernel.getPredicate('emotion', session)
        ret['botid'] = self.id
        ret['botname'] = self.name
        return ret

class SheetAIMLCharacter(AIMLCharacter):
    def __init__(self, id, name, level):
        super(SheetAIMLCharacter, self).__init__(id, name, level)
        self.sheet_keys = []
        self.set_property_file(os.path.join(
                        CWD, "../character_aiml/{}.properties".format(name)))
        self.aiml_files = []
        self.incoming_aiml_files = []
        self.csv_files = []
        self.commited_aiml_files = []
        self.incoming_dir = os.path.expanduser(
            '~/.hr/aiml/{}/incoming'.format(self.id))
        self.commit_dir = os.path.expanduser(os.path.join(GIT_DIR, 'commit'))
        if not os.path.isdir(self.incoming_dir):
            os.makedirs(self.incoming_dir)
        if not os.path.isdir(self.commit_dir):
            os.makedirs(self.commit_dir)
        self.csv_dir = self.incoming_dir

    def load_sheet_keys(self, keys_str):
        self.sheet_keys = keys_str.strip().split(',')
        if not self.sheet_keys:
            logger.error("No sheet keys")
            return False, "No sheet keys"
        self._load_sheet_keys()
        self.reload_character()
        return True, "Sheet keys are loaded"

    def _load_sheet_keys(self):
        self.aiml_files = []
        for key in self.sheet_keys:
            logger.info("Loading sheet key {} from Google Sheets".format(key))
            aiml_files, csvs = getWorkSheets(key, self.incoming_dir)
            if not aiml_files:
                logger.warn("No sheet is found with key {}".format(key))
                continue
            self.aiml_files.extend(aiml_files)
            self.csv_files.extend(csvs)
        if not self.aiml_files:
            logger.warn("No aiml files to load")
            return

    def set_csv_dir(self, csv_dir):
        self.csv_dir = csv_dir

    def load_csv_files(self, csv_version=None):
        self.incoming_aiml_files, self.csv_files = batch_csv2aiml(
            self.csv_dir, self.csv_dir, csv_version)
        self.reload_character()

    def reload_character(self):
        kernel = aiml.Kernel()
        committed_aimls = []
        for root, _, files in os.walk(self.commit_dir):
            committed_aimls.extend(
                [os.path.join(root, f) for f in files if f.endswith('.aiml')])

        logger.info("Reloading character {}".format(self.id))
        skip_aimls = [os.path.basename(f) for f in self.incoming_aiml_files]
        self.load_aiml_files(kernel, [f for f in committed_aimls
                            if os.path.basename(f) not in skip_aimls])
        self.load_aiml_files(kernel, self.incoming_aiml_files)
        del(self.kernel)
        self.kernel = kernel
        logger.info("Reloaded character {}".format(self.id))

    def commit(self):
        if not os.path.isdir(self.csv_dir):
            return False, "Nothing to commit."
        dest = os.path.join(self.commit_dir)
        dir_to_commit = os.path.join(dest, os.path.basename(self.csv_dir))
        shutil.rmtree(dir_to_commit, True)
        shutil.move(self.csv_dir, dest)

        user = 'default'
        email = ''
        user , _ = subprocess.Popen(
            'git config --get user.name', shell=True, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE).communicate()
        email, _ = subprocess.Popen(
            'git config --get user.email', shell=True, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE).communicate()
        if '@' in self.id:
            email = self.id
            user = email.split('@')[0].title()
        user = user.strip()
        email = email.strip()

        cmd = '''cd {} && git add -A {} && git -c user.name="{}" -c user.email={} commit -m "{}" && git push origin {}'''.format(
            self.commit_dir, dir_to_commit, user, email, COMMENT, WORKING_BRANCH)
        stdout, stderr = subprocess.Popen(
            cmd, shell=True, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE).communicate()
        output = stdout + stderr
        logger.info("Commit character {}".format(self.id))
        logger.info("Commit cmd {}".format(cmd))
        if 'failed' in output or 'nothing to commit' in output:
            ret = False
        else:
            ret = True
        logger.info(output)
        self.incoming_aiml_files, self.csv_files = [], []
        self.reload_character()
        return ret, output
