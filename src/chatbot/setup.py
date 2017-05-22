# DO NOT USE
# python setup.py install

from distutils.core import setup

setup(
    version='0.1.1',
    name='chatbot',
    packages=['chatbot', 'chatbot.server', 'chatbot.aiml'],
    package_dir={'': 'src'}
)
