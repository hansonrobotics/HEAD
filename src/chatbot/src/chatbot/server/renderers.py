import time
import json
from chatbot.utils import *

def render_weather(t):
    if hasattr(t.module, 'location'):
        location = t.module.location
    else:
        location = get_location()
        if location:
            if 'city' in location:
                location = location.get('city')
    city = query_city_info(location)
    if 'id' in city:
        id = city['id']
        weather = get_weather_by_id(id)
        weather_prop = parse_weather(weather)
        desc = weather_prop['weather']
        temperature = '{} degrees'.format(weather_prop['temperature'])
        return t.render(weatherdesc=desc, temperature=temperature, location=city)

def render_location(t):
    location = get_location()
    if location:
        if 'city' in location:
            location = location.get('city')
            return t.render(location=location)

def render_face_emotion(t):
    time.sleep(3)
    emotion = get_emotion(3) # the the emotion in the past 3 secs
    if emotion is not None:
        return t.render(faceemotion=emotion)

def render_object_detected(t):
    time.sleep(3)
    item = get_detected_object(10)
    if item is not None:
        item = ' '.join(item.split('_'))
        return t.render(objectdetected=item)
