import json
from chatbot.utils import get_location, get_weather_by_id, parse_weather, query_city_info

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
