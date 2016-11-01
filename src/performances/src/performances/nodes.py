#!/usr/bin/env python

# Nodes factory
import os
import pprint
import rospy
from std_msgs.msg import String, Int32, Float32
from chatbot.msg import ChatMessage
from blender_api_msgs.msg import SetGesture, EmotionState, Target, SomaState
from basic_head_api.msg import MakeFaceExpr, PlayAnimation
from topic_tools.srv import MuxSelect
import time
import logging
import random
from threading import Timer
from performances.msg import Event
import requests
import urllib
import dynamic_reconfigure.client
import re
from performances.srv import RunByNameRequest

logger = logging.getLogger('hr.performances.nodes')


class Node(object):
    # Create new Node from JSON
    @staticmethod
    def subClasses(cls):
        return cls.__subclasses__() + [g for s in cls.__subclasses__()
                                       for g in cls.subClasses(s)]

    @classmethod
    def createNode(cls, data, runner, start_time=0, id=''):
        for s_cls in cls.subClasses(cls):
            if data['name'] == s_cls.__name__:
                node = s_cls(data, runner)
                node.id = id
                if start_time > node.start_time:
                    # Start time should be before or on node starting
                    node.finished = True
                return node
        print "Wrong node description"

    def __init__(self, data, runner):
        self.data = data
        self.duration = data['duration']
        self.start_time = data['start_time']
        self.started = False
        self.started_at = 0
        self.finished = False
        self.id = ''
        # Node runner for accessing ROS topics and method
        # TODO make ROS topics and services singletons class for shared use.
        self.runner = runner

    # By default end time is started + duration for every node
    def end_time(self):
        return self.start_time + self.duration

    # Manages node states. Currently start, finish is implemented.
    # Returns True if its active, and False if its inactive.
    # TODO make sure to allow node publishing pause and stop
    def run(self, run_time):
        # ignore the finished nodes
        if self.finished:
            return False
        if self.started:
            # Time to finish:
            if run_time >= self.end_time():
                self.stop(run_time)
                self.finished = True
                return False
            elif self.runner.paused:
                self.paused(run_time)
            else:
                self.cont(run_time)
        else:
            if run_time > self.start_time:
                try:
                    self.start(run_time)
                except Exception as ex:
                    logger.error(ex)
                self.started = True
                self.started_at = time.time()
        return True

    def __str__(self):
        return pprint.pformat(self.data)

    # Method to execute if node needs to start
    def start(self, run_time):
        pass

    # Method to execute while node is stopping
    def stop(self, run_time):
        pass

    # Method to call while node is running
    def cont(self, run_time):
        pass

    # Method to call while runner is paused
    def paused(self, run_time):
        pass

    # Method to get magnitude from either one number or range
    @staticmethod
    def _magnitude(magnitude):
        try:
            return float(magnitude)
        except TypeError:
            try:
                # Randomize magnitude
                return random.uniform(float(magnitude[0]), float(magnitude[1]))
            except:
                return 0.0


class speech(Node):
    def __init__(self, data, runner):
        Node.__init__(self, data, runner)
        if 'pitch' not in data:
            self.data['pitch'] = 1.0
        if 'speed' not in data:
            self.data['speed'] = 1.0
        if 'volume' not in data:
            self.data['volume'] = 1.0

    def start(self, run_time):
        self.say(self.data['text'], self.data['lang'])

    def say(self, text, lang):
        if lang not in ['en', 'zh']:
            lang = 'default'
        # SSML tags for english TTS only.
        if lang == 'en':
            text = self._add_ssml(text)
        variables = re.findall("{(.*?)}", text)
        for var in variables:
            val = self.runner.get_property(self.id, var) or ''
            text = text.replace('{' + var + '}', val)
        self.runner.topics['tts'][lang].publish(String(text))

    # adds SSML tags for whole text returns updated text.
    def _add_ssml(self, txt):
        return '<prosody rate="%.2f" pitch="%+d%%" volume="%+d%%">%s</prosody>' % \
               (self.data['speed'], 100 * (self.data['pitch'] - 1), 100 * (self.data['volume'] - 1), txt)


class gesture(Node):
    def start(self, run_time):
        self.runner.topics['gesture'].publish(
            SetGesture(self.data['gesture'], 1, float(self.data['speed']), self._magnitude(self.data['magnitude'])))


class emotion(Node):
    def start(self, run_time):
        self.runner.topics['emotion'].publish(
            EmotionState(self.data['emotion'], self._magnitude(self.data['magnitude']),
                         rospy.Duration.from_sec(self.data['duration'])))


# Behavior tree
class interaction(Node):
    def start(self, run_time):
        self.runner.topics['bt_control'].publish(Int32(self.data['mode']))
        if self.data['chat'] == 'listening':
            self.runner.topics['speech_events'].publish(String('listen_start'))
        if self.data['chat'] == 'talking':
            self.runner.topics['speech_events'].publish(String('start'))
        time.sleep(0.02)
        self.runner.topics['interaction'].publish(String('btree_on'))

    def stop(self, run_time):
        # Disable all outputs
        self.runner.topics['bt_control'].publish(Int32(0))

        if self.data['chat'] == 'listening':
            self.runner.topics['speech_events'].publish(String('listen_stop'))
        if self.data['chat'] == 'talking':
            self.runner.topics['speech_events'].publish(String('stop'))
        time.sleep(0.02)
        self.runner.topics['interaction'].publish(String('btree_off'))


# Rotates head by given angle
class head_rotation(Node):
    def start(self, run_time):
        self.runner.topics['head_rotation'].publish(Float32(self.data['angle']))


class soma(Node):
    def start(self, run_time):
        s = SomaState()
        s.magnitude = 1
        s.ease_in.secs = 0
        s.ease_in.nsecs = 1000000 * 300
        s.name = self.data['soma']
        self.runner.topics['soma_state'].publish(s)

    def stop(self, run_time):
        s = SomaState()
        s.magnitude = 0
        s.ease_in.secs = 0
        s.ease_in.nsecs = 0
        s.name = self.data['soma']
        self.runner.topics['soma_state'].publish(s)


class expression(Node):
    def __init__(self, data, runner):
        Node.__init__(self, data, runner)
        self.shown = False

    def start(self, run_time):
        try:
            self.runner.services['head_pau_mux']("/" + self.runner.robot_name + "/no_pau")
            logger.info("Call head_pau_mux topic {}".format("/" + self.runner.robot_name + "/no_pau"))
        except Exception as ex:
            logger.error(ex)
        self.shown = False

    def cont(self, run_time):
        # Publish expression message after some delay once node is started
        if (not self.shown) and (run_time > self.start_time + 0.05):
            self.shown = True
            self.runner.topics['expression'].publish(
                MakeFaceExpr(self.data['expression'], self._magnitude(self.data['magnitude'])))
            logger.info("Publish expression {}".format(self.data))

    def stop(self, run_time):
        try:
            self.runner.services['head_pau_mux']("/blender_api/get_pau")
            logger.info("Call head_pau_mux topic {}".format("/blender_api/get_pau"))
        except Exception as ex:
            logger.error(ex)


class kfanimation(Node):
    def __init__(self, data, runner):
        Node.__init__(self, data, runner)
        self.shown = False
        self.blender_disable = 'off'
        if 'blender_mode' in self.data.keys():
            self.blender_disable = self.data['blender_mode']

    def start(self, run_time):
        self.shown = False
        try:
            if self.blender_disable in ['face', 'all']:
                self.runner.services['head_pau_mux']("/" + self.runner.robot_name + "/no_pau")
            if self.blender_disable == 'all':
                self.runner.services['neck_pau_mux']("/" + self.runner.robot_name + "/cmd_neck_pau")
        except Exception as ex:
            # Dont start animation to prevent the conflicts
            self.shown = True
            logger.error(ex)

    def cont(self, run_time):
        # Publish expression message after some delay once node is started
        if (not self.shown) and (run_time > self.start_time + 0.05):
            self.shown = True
            self.runner.topics['kfanimation'].publish(
                PlayAnimation(self.data['animation'], int(self.data['fps'])))

    def stop(self, run_time):
        try:
            if self.blender_disable in ['face', 'all']:
                self.runner.services['head_pau_mux']("/blender_api/get_pau")
            if self.blender_disable == 'all':
                self.runner.services['neck_pau_mux']("/blender_api/get_pau")
        except Exception as ex:
            logger.error(ex)


class pause(Node):
    def __init__(self, data, runner):
        Node.__init__(self, data, runner)
        self.subscriber = False
        self.timer = False
        if 'on_event' not in self.data.keys():
            self.data['on_event'] = False
        if 'event_param' not in self.data.keys():
            self.data['event_param'] = False

    def start_performance(self):
        if self.subscriber:
            self.runner.unregister(self.data['topic'].strip(), self.subscriber)
            self.subscriber = None
        req = RunByNameRequest()
        req.id = self.data['on_event'].strip()
        self.timer.cancel()
        response = self.runner.run_full_performance_callback(req)
        if not response.success:
            self.runner.resume()

    def event_callback(self, msg=None):
        if self.data['event_param']:
            # Check if any comma separated
            params = str(self.data['event_param']).lower().split(',')
            matched = False
            for p in params:
                try:
                    str(msg or '').lower().index(p.strip())
                    matched = True
                    continue
                except ValueError:
                    matched = matched or False
            if not matched:
                return False
        if self.data['on_event']:
            self.start_performance()
        else:
            self.resume()

    def resume(self):
        if self.subscriber:
            self.runner.unregister(self.data['topic'].strip(), self.subscriber)
            self.subscriber = None
        if not self.finished:
            self.runner.resume()
        if self.timer:
            self.timer.cancel()

    def start(self, run_time):
        self.runner.pause()
        if 'topic' in self.data:
            topic = self.data['topic'].strip()
            self.subscriber = self.runner.register(topic, self.event_callback)
        try:
            timeout = float(self.data['timeout'])
            if timeout > 0.1:
                self.timer = Timer(timeout, self.resume)
                self.timer.start()
        except (ValueError, KeyError) as e:
            logger.error(e)

    def stop(self, run_time):
        if self.subscriber:
            self.runner.unregister(self.data['topic'].strip(), self.subscriber)
            self.subscriber = None
        if self.timer:
            self.timer.cancel()

    def end_time(self):
        return self.start_time + 0.1


class chat_pause(Node):
    def __init__(self, data, runner):
        Node.__init__(self, data, runner)
        self.subscriber = False

    def start(self, run_time):
        if 'message' in self.data and self.data['message']:
            self.runner.pause()
            self.runner.topics['chatbot'].publish(ChatMessage(self.data['message'], 100))

            def speech_event_callback(event):
                if event.data == 'stop':
                    self.resume()

            self.subscriber = rospy.Subscriber('/' + self.runner.robot_name + '/speech_events', String,
                                               speech_event_callback)

            while not self.finished and self.runner.start_timestamp + self.start_time + self.duration > time.time():
                time.sleep(0.05)
        self.resume()

    def resume(self):
        self.duration = 0
        self.runner.resume()

    def stop(self, run_time):
        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = False


class chat(Node):
    def __init__(self, data, runner):
        Node.__init__(self, data, runner)
        self.subscriber = False
        self.turns = 0
        self.last_turn_at = 0
        self.chatbot_session_id = False
        self.enable_chatbot = 'enable_chatbot' in self.data and self.data['enable_chatbot']
        self.talking = False

        try:
            self.dialog_turns = int(self.data['dialog_turns'])
        except (ValueError, KeyError):
            self.dialog_turns = 1

        try:
            self.timeout = float(self.data['timeout'])
        except (ValueError, KeyError):
            self.timeout = 0

        try:
            self.timeout_mode = self.data['timeout_mode']
        except:
            self.timeout_mode = 'each'

    def start(self, run_time):
        self.runner.pause()
        self.last_turn_at = time.time()

        if self.enable_chatbot:
            self.start_chatbot_session()

        def input_callback(event):
            self.respond(
                self.get_chatbot_response(event.data) if self.enable_chatbot else self.match_response(event.data))

        self.subscriber = rospy.Subscriber('/' + self.runner.robot_name + '/nodes/listen/input', String, input_callback)
        self.runner.topics['events'].publish(Event('chat', 0))
        self.runner.register('speech_events', self.speech_event_callback)

    def stop(self, run_time):
        self.runner.unregister('speech_events', self.speech_event_callback)

    def paused(self, run_time):
        if self.timeout and not self.talking:
            if (self.timeout_mode == 'each' and time.time() - self.last_turn_at >= self.timeout) or (
                            self.timeout_mode == 'whole' and time.time() - self.started_at >= self.timeout):
                if 'no_speech' in self.data:
                    self.respond(self.data['no_speech'])
                else:
                    self.add_turn()

    def start_chatbot_session(self):
        params = {
            'Auth': 'AAAAB3NzaC',
            'botname': self.data['bot_name'],
            'user': 'performances'
        }

        r = requests.get('http://127.0.0.1:8001/v1.1/start_session?' + urllib.urlencode(params))

        if r.status_code == 200:
            self.chatbot_session_id = r.json()['sid']

    def get_chatbot_response(self, speech):
        if self.chatbot_session_id:
            params = {
                'Auth': 'AAAAB3NzaC',
                'lang': 'en',
                'question': speech,
                'session': self.chatbot_session_id
            }

            r = requests.get('http://127.0.0.1:8001/v1.1/chat?' + urllib.urlencode(params))
            if r.status_code == 200:
                return r.json()['response']['text']

        return ''

    def match_response(self, speech):
        response = ''
        if 'responses' in self.data and isinstance(self.data['responses'], list):
            input = speech.lower()
            matches = []
            for r in self.data['responses']:
                if r['input'] in input:
                    matches.append(r['output'])

            if len(matches):
                response = matches[int(random.randint(0, len(matches) - 1))]

        if not response and 'no_match' in self.data:
            response = self.data['no_match']

        return response

    def add_turn(self):
        self.turns += 1
        self.last_turn_at = time.time()

    def resume(self):
        self.duration = 0
        self.runner.resume()
        self.runner.topics['events'].publish(Event('chat_end', 0))

        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = False

    def respond(self, response):
        self.runner.topics['events'].publish(Event('chat_end', 0))
        self.talking = True
        self.runner.topics['tts']['default'].publish(String(response))

    def speech_event_callback(self, msg):
        event = msg.data

        if event == 'stop':
            self.add_turn()
            self.talking = False

            if self.turns < self.dialog_turns:
                self.runner.topics['events'].publish(Event('chat', 0))
            else:
                self.resume()


class attention(Node):
    # Find current region at runtime
    def __init__(self, data, runner):
        Node.__init__(self, data, runner)
        self.topic = 'look_at'
        self.times_shown = 0

    def get_random_axis_position(self, regions, axis):
        """
        :param regions: list of dictionaries
        :param axis: string 'x' or 'y'
        :return: position and matched regions
        """

        position = 0
        matched = []

        if regions:
            regions = sorted(regions, key=lambda r: r[axis])
            prev_end = regions[0][axis]
            length = 0
            lengths = []

            for r in regions:
                begin = r[axis]
                end = begin + (r['width'] if axis == 'x' else r['height'])

                if prev_end > begin:
                    diff = prev_end - begin
                    lengths.append([length - diff, length - diff + end - begin])
                    begin = prev_end
                else:
                    lengths.append([length, length + end - begin])
                length += max(0, end - begin)
                prev_end = max(begin, end)

            rval = random.random() * length

            for i, length in enumerate(lengths):
                if length[0] <= rval <= length[1]:
                    matched.append(regions[i])
                    if not position:
                        position = regions[i][axis] + (regions[i]['width'] if axis == 'x' else regions[i]['height']) * (
                            (rval - length[0]) / (length[1] - length[0]))

        return position, matched

    # returns random coordinate from the region
    def get_point(self, region):
        regions = rospy.get_param('/' + self.runner.robot_name + "/attention_regions")
        regions = [{'x': r['x'], 'y': r['y'] - r['height'], 'width': r['width'], 'height': r['height']} for r in regions
                   if r['type'] == region]

        if regions:
            y, matched = self.get_random_axis_position(regions, 'x')
            z, matched = self.get_random_axis_position(matched, 'y')

            return {
                'x': 1,
                'y': y,
                'z': z,
            }
        else:
            # Look forward
            return {'x': 1, 'y': 0, 'z': 0}

    def set_point(self, point):
        speed = 1 if 'speed' not in self.data else self.data['speed']
        self.runner.topics[self.topic].publish(Target(point['x'], point['y'], point['z'], speed))

    def cont(self, run_time):
        if 'attention_region' in self.data and self.data['attention_region'] != 'custom':
            if 'interval' in self.data and run_time > self.times_shown * self.data['interval'] or not self.times_shown:
                self.set_point(self.get_point(self.data['attention_region']))
                self.times_shown += 1

        if not self.times_shown:
            self.set_point(self.data)
            self.times_shown += 1


class look_at(attention):
    # Find current region at runtime
    def __init__(self, data, runner):
        attention.__init__(self, data, runner)
        self.topic = 'look_at'


class gaze_at(attention):
    # Find current region at runtime
    def __init__(self, data, runner):
        attention.__init__(self, data, runner)
        self.topic = 'gaze_at'


class settings(Node):
    def setParameters(self, rosnode, params):
        cl = dynamic_reconfigure.client.Client(rosnode)
        res = cl.update_configuration(params)
        cl.close()

    def start(self, run_time):
        if (self.data['rosnode']):
            self.setParameters(self.data['rosnode'], self.data['values'])
