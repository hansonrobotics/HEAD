# DO NOT USE
# python setup.py install

from distutils.core import setup

setup(
    version='0.0.1',
    packages=['chatbot', 'chatbot.server', 'chatbot.aiml'],
    package_dir={'': 'src'}
)
