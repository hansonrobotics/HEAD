import requests
import os
import datetime as dt

import sys
reload(sys)
sys.setdefaultencoding('utf-8')

class Client(object):

    VERSION = 'v1.0'

    def __init__(self):
        self.host = 'http://localhost'
        self.port = '9001'
        self.root_url = '{}:{}/{}'.format(self.host, self.port, Client.VERSION)

    def detect_faces(self, image_file):
        """
        image_file: file-like object
        """
        try:
            r = requests.post(
                '{}/detect_face'.format(self.root_url), data=image_file)
            ret = r.json().get('ret')
            response = r.json().get('response')
            return response
        except Exception as ex:
            print ex


if __name__ == '__main__':
    client = Client()
    start = dt.datetime.now()
    for i in range(100):
        with open('obama.png', 'rb') as f:
            print client.detect_faces(f)
            print dt.datetime.now() - start
