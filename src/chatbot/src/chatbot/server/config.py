import os

DEFAULT_CHARACTER_PATH = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), 'characters')
CHARACTER_PATH = os.environ.get('HR_CHARACTER_PATH', DEFAULT_CHARACTER_PATH)

SESSION_REMOVE_TIMEOUT = 600  # Timeout seconds for a session to be removed

CHATBOT_LOG_DIR = os.environ.get('CHATBOT_LOG_DIR') or os.path.expanduser('~/.hr/chatbot')
HISTORY_DIR = os.path.join(CHATBOT_LOG_DIR, 'history')
TEST_HISTORY_DIR = os.path.join(CHATBOT_LOG_DIR, 'test/history')

SOLR_URL = 'http://localhost:8983'

HR_CHATBOT_AUTHKEY = os.environ.get('HR_CHATBOT_AUTHKEY', 'AAAAB3NzaC')
