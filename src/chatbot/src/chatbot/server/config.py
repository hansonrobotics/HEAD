import os

DEFAULT_CHARACTER_PATH = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), 'characters')
CHARACTER_PATH = os.environ.get('HR_CHARACTER_PATH', DEFAULT_CHARACTER_PATH)

RESET_SESSION_BY_HELLO = False
SESSION_REMOVE_TIMEOUT = 600  # Timeout seconds for a session to be removed

CHATBOT_LOG_DIR = os.environ.get('CHATBOT_LOG_DIR') or os.path.expanduser('~/.hr/chatbot')
SERVER_LOG_DIR = os.environ.get('SERVER_LOG_DIR') or os.path.expanduser('~/.hr/log/chatbot')
HISTORY_DIR = os.path.join(CHATBOT_LOG_DIR, 'history')
TEST_HISTORY_DIR = os.path.join(CHATBOT_LOG_DIR, 'test/history')
CS_HOST = os.environ.get('CS_HOST') or 'localhost'
CS_PORT = os.environ.get('CS_PORT') or '1024'
CS_BOT = os.environ.get('CS_BOT') or 'rose'

HR_CHATBOT_AUTHKEY = os.environ.get('HR_CHATBOT_AUTHKEY', 'AAAAB3NzaC')

config = {}
config['DEFAULT_CHARACTER_PATH'] = DEFAULT_CHARACTER_PATH
config['CHARACTER_PATH'] = CHARACTER_PATH
config['RESET_SESSION_BY_HELLO'] = RESET_SESSION_BY_HELLO
config['SESSION_REMOVE_TIMEOUT'] = SESSION_REMOVE_TIMEOUT
config['CHATBOT_LOG_DIR'] = CHATBOT_LOG_DIR
config['SERVER_LOG_DIR'] = SERVER_LOG_DIR
config['HISTORY_DIR'] = HISTORY_DIR
config['CS_HOST'] = CS_HOST
config['CS_PORT'] = CS_PORT
config['CS_BOT'] = CS_BOT
config['HR_CHATBOT_AUTHKEY'] = HR_CHATBOT_AUTHKEY
