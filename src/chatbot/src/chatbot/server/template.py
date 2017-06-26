import os
from jinja2 import Template, Environment, meta
from renderers import render_weather, render_location

def render(string):
    t = Template(string)
    func = get_render_func(string)
    if func is not None:
        rendered_text = func(t)
        return rendered_text

def get_render_func(string):
    func = None
    env = Environment()
    ast = env.parse(string)
    variables = meta.find_undeclared_variables(ast)
    if 'temperature' in variables:
        func = render_weather
    if 'location' in variables:
        func = render_location
    return func

if __name__ == '__main__':
    string = """{% set location = "hong kong" %} {% if temperature is not defined %} I don't know {% elif temperature>30 %} it's really hot {% elif temperature<10 %} too cold here {% else %} Hmm, it\'s nice {% endif %}."""
    string  = render(string)
    print string

    string = """{% if temperature is not defined %} I don't know {% else %} The weather is {{ weather_desc }}, the temperature is {{ temperature }}. {% endif %}"""
    string  = render(string)
    print string

    string = """I think you are in {{ location }}. """
    string  = render(string)
    print string
