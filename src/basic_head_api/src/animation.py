__author__ = 'vytas'
from copy import *
from collections import OrderedDict
from MotorCmder import MotorCmder


class Animation:
    def __init__(self, conf):
        self._keyframes = conf
        self._current = 0
        self._timeline = {}
        self._build_timeline()

    def _build_timeline(self):
        #  Creates animation actions for each motor
        current_frame = 0
        for k in self._keyframes:
            for m, v in k['motors'].items():
                if m in self._timeline.keys():
                    # Animation action starts where previous ended
                    act = {
                        'start': self._timeline[m][-1]['stop'],
                        'prev': self._timeline[m][-1]['target'],
                        'stop': current_frame + k['frames'],
                        'target': v
                    }
                    self._timeline[m].append(act)
                else:
                    # Motor is added with initial value.
                    # All motors used in animations are reset after animation starts
                    act = {
                        'start': 0,
                        'prev': v,
                        'stop': current_frame + k['frames'],
                        'target': v
                    }
                    self._timeline[m] = [act]

            current_frame += k['frames']
        self.total = current_frame

    # currently linear
    @staticmethod
    def interp(action, frame):
        k = (frame - action['start']) / float(action['stop'] - action['start'])
        return action['prev'] + k * (action['target'] - action['prev'])

    def frames(self):
        timeline = deepcopy(self._timeline)
        for x in xrange(self.total):
            current = {}
            for m, actions in timeline.items():
                if len(actions) > 0:
                    current[m] = self.interp(actions[0], x + 1)
                    # Remove finished actions
                    if actions[0]['stop'] == x + 1:
                        del timeline[m][0]
            yield current

    @classmethod
    def from_yaml(cls, animations):
        result = OrderedDict()
        for anim in animations:
            result[anim] = cls(animations[anim])
        return result


if __name__ == '__main__':
    import yaml
    import os
    import rospkg
    path = rospkg.RosPack().get_path('robots_config')
    animation_file = '%s/arthur/animations.yaml' % path
    anims = open(animation_file)
    y = yaml.load(anims)
    a = Animation(y['animations'][0]['Happy'])
    for f in a.frames():
        print f

