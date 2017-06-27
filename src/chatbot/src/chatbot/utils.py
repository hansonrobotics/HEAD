import re
import os
import requests
import subprocess
import logging
import json

logger = logging.getLogger('hr.chatbot.utils')

OPENWEATHERAPPID = os.environ.get('OPENWEATHERAPPID')
CITY_LIST_FILE = os.environ.get('CITY_LIST_FILE')

cities = None

def query_city_info(name):
    global cities
    if cities is None:
        if CITY_LIST_FILE:
            with open(CITY_LIST_FILE) as f:
                cities = json.load(f)
    for city in cities:
        if name.title() in city['name']:
            return city

def str_cleanup(text):
    if text:
        text = text.strip()
        text = ' '.join(text.split())
        if text and text[0] == '.':
            text = text[1:]
    return text

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

    res = '.'.join(sens[idx:])

    # If first part or second part is too short, then don't cut
    if len(ret.split()) < 3 or len(res.split()) < 3:
        ret = text
        res = ''

    ret = str_cleanup(ret)
    res = str_cleanup(res)
    return ret, res

def get_location():
    # docker run -d -p 8004:8004 --name freegeoip fiorix/freegeoip -http :8004
    host = os.environ.get('LOCATION_SERVER_HOST', 'localhost')
    port = os.environ.get('LOCATION_SERVER_PORT', '8004')
    location = None
    try:
        ip = subprocess.check_output(['dig', '+short', 'myip.opendns.com', '@resolver1.opendns.com']).strip()
        location = requests.get('http://{host}:{port}/json/{ip}'.format(host=host, port=port, ip=ip)).json()
        if location['country_code'] == 'HK':
            location['city'] = 'Hong Kong'
        if location['country_code'] == 'TW':
            location['city'] = 'Taiwan'
        if location['country_code'] == 'MO':
            location['city'] = 'Macau'
    except Exception as ex:
        pass
    return location

def get_weather(city):
    if city:
        try:
            response = requests.get(
                'http://api.openweathermap.org/data/2.5/weather',
                timeout=5,
                params={'q': city, 'appid': OPENWEATHERAPPID}).json()
        except Exception as ex:
            logger.error(ex)
            return
        return response

def get_weather_by_id(city_id):
    if city_id:
        try:
            response = requests.get(
                'http://api.openweathermap.org/data/2.5/weather',
                timeout=5,
                params={'id': city_id, 'appid': OPENWEATHERAPPID}).json()
        except Exception as ex:
            logger.error(ex)
            return
        return response

def parse_weather(weather):
    kelvin = 273.15
    prop = {}
    if weather and weather['cod'] == 200:
        if 'main' in weather:
            if 'temp_max' in weather['main']:
                prop['high_temperature'] = \
                    '{:.0f}'.format(weather['main'].get('temp_max')-kelvin)
            if 'temp_min' in weather['main']:
                prop['low_temperature'] = \
                    '{:.0f}'.format(weather['main'].get('temp_min')-kelvin)
            if 'temp' in weather['main']:
                prop['temperature'] = \
                    '{:.0f}'.format(weather['main'].get('temp')-kelvin)
        if 'weather' in weather and weather['weather']:
            prop['weather'] = weather['weather'][0]['description']
    return prop

def check_online(url, port='80'):
    try:
        subprocess.check_call(['nc', '-z', '-w', '1', str(url), str(port)])
    except Exception as ex:
        logger.error(ex)
        return False
    return True

if __name__ == '__main__':
    logging.basicConfig()
    text = '''My mind is built using Hanson Robotics' character engine, a simulated humanlike brain that runs inside a personal computer. Within this framework, Hanson has modelled Phil's personality and emotions, allowing you to talk with Phil through me, using speech recognition, natural language understanding, and computer vision such as face recognition, and animation of the robotic muscles in my face.'''
    print len(text)
    print text
    print shorten(text, 123)

    text = '''My mind is built using Hanson Robotics' character engine'''
    print len(text)
    print text
    print shorten(text, 123)

    print str_cleanup('.')
    print str_cleanup(' .ss ')
    print str_cleanup(' s.ss ')
    print str_cleanup('')
    print str_cleanup(None)
    print check_online('google.com')
    print check_online('duckduckgo.com', 80)
