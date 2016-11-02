import rospy
import os

WHOLE_SHOW_BUILTIN_COMMANDS = [
    'go to sleep',
    'wake', 'makeup',
    'shutdown',
    'be quiet',
    'hi sophia', 'hey sophia', 'hello sofia', 'hello sophia', 'hi sofia', 'hey sofia'
]

def get_keywords(performances=None, keywords=None, path='.'):
    if performances is None:
        performances = rospy.get_param(os.path.join('/', rospy.get_param('/robot_name'), 'webui/performances'))
        keywords = {}

    if 'properties' in performances and 'keywords' in performances['properties']:
        keywords[path] = performances['properties']['keywords']

    for key, value in performances.items():
        if key != 'properties':
            get_keywords(performances[key], keywords, os.path.join(path, key).strip('./'))

    return keywords

if __name__ == '__main__':
    print sum(get_keywords().values(), [])
