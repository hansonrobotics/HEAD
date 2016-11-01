import re
import os
import requests
import subprocess


def norm(s):
    if s is None:
        return s
    s = re.sub(r'\[.*\]', '', s) # remote [xxx] mark
    s = ' '.join(s.split())  # remove consecutive spaces
    s = s.strip()
    return s

def shorten(text, cutoff):
    if len(text) < cutoff:
        return text, ''
    sens = text.split('.')
    ret = ''
    for idx, sen in enumerate(sens):
        if len(ret) > 0 and len(ret+sen) > cutoff:
            break
        ret += (sen+'.')
    return ret, '.'.join(sens[idx:])

def get_location():
    # docker run --net=host --restart=always -d fiorix/freegeoip
    ip = subprocess.check_output(['dig', '+short', 'myip.opendns.com', '@resolver1.opendns.com']).strip()
    location = requests.get('http://localhost:8080/json/{ip}'.format(ip=ip)).json()
    return location

def get_weather(city):
    OPENWEATHERAPPID = os.environ.get('OPENWEATHERAPPID')
    response = requests.get('http://api.openweathermap.org/data/2.5/weather', params={'q': city, 'appid': OPENWEATHERAPPID}).json()
    return response

if __name__ == '__main__':
    text = '''My mind is built using Hanson Robotics' character engine, a simulated humanlike brain that runs inside a personal computer. Within this framework, Hanson has modelled Phil's personality and emotions, allowing you to talk with Phil through me, using speech recognition, natural language understanding, and computer vision such as face recognition, and animation of the robotic muscles in my face.'''
    print len(text)
    print text
    print shorten(text, 123)

    text = '''My mind is built using Hanson Robotics' character engine'''
    print len(text)
    print text
    print shorten(text, 123)

