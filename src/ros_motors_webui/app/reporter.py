import yaml
from subprocess import call, check_output
from easyprocess import EasyProcess
import threading
import time
from ros_pololu import PololuMotor
from configs import Configs
import math
import psutil

try:
    from subprocess import DEVNULL
except ImportError:
    import os
    DEVNULL = open(os.devnull, 'wb')

class Reporter:
    # Executes a number of shell commands, interprets their return codes as
    # success or failure and returns results.
    def __init__(self, yamlfilename):
        self.filename = yamlfilename
        self.load_config()
        self.env = None
        # Dynamixel status updates
        self.monitor_started = False
        self.dynamixel_motors_states = {}
        self.robot_name = ''
        # Pololu status
        self.pololu_boards = {}

    def load_config(self):
        with open(self.filename, 'r') as f:
            config = yaml.load(f)

        # Set default values in case config['setup']
        # or config['setup']['env'], etc. are None.
        defaults = {'setup': {'env': {}, 'prepend': []}}
        deepupdate(config, defaults)
        self.config = config

    def report(self):
        # Reload config file
        self.load_config()

        # Execute and store environment variables
        self.env = self._build_env()
        # Build commands to prepend to checks
        cmdlist = self.config['setup']['prepend']

        # Do the checks
        statuslist = []
        for check in self.config['checks']:
            cmd = '; '.join(cmdlist + [check['cmd']])
            status = self.check(cmd, env=self.env)
            statuslist.append(status)

        # Return a copy of self.config['checks'] with a new parameter 'success'
        return [dict(list(check.items()) + [('success', success)])
                for check, success in zip(self.config['checks'], statuslist)]

    # Adds the states to the motors:
    # 0 - Running
    # 1 - Error
    # 2 - No Feedback
    def motor_states(self, motors, robot_name):
        # Dynamixel monitor start
        if not self.monitor_started:
            for i,m in enumerate(motors):
                if m['hardware'] == 'pololu':
                    self.pololu_boards[m['topic']] = {}
            self.start_motors_monitor(robot_name)
            self.monitor_started = True
            # Sleep some time so first results if motors are alive will have time to return
            time.sleep(0.5)
        status = {}
        pololu_boards = {}
        for i, m in enumerate(motors):
            if m['hardware'] == 'pololu':
                if not m['topic'] in self.pololu_boards.keys():
                    self.pololu_boards[m['topic']] = {}
                if m['name'] in self.pololu_boards[m['topic']].keys():
                    m['motor_state'] = {'position': self.pololu_boards[m['topic']][m['name']]}
                    m['error'] = 2
                else:
                    m['error'] = 1
                motor = PololuMotor(m['name'], m)
                #Conert to angles
                m['init'] = round(math.degrees(motor.get_angle(m['init']*4)))
                m['min'] = round(math.degrees(motor.get_angle(m['min']*4)))
                m['max'] = round(math.degrees(motor.get_angle(m['max']*4)))
            #Dynamixel motors
            else:
                if m['motor_id'] in self.dynamixel_motors_states.keys():
                    motors[i]['error'] = 0
                    motors[i]['motor_state'] = self.dynamixel_motors_states[m['motor_id']]
                else:
                    # Motor is not on
                    motors[i]['error'] = 1
                m['max'] = round(math.degrees(Configs.dynamixel_angle(m, m['max'])))
                m['min'] = round(math.degrees(Configs.dynamixel_angle(m, m['min'])))
                # Init has to be replaced last, because calculation depends on it
                m['init'] = round(math.degrees(Configs.dynamixel_angle(m, m['init'])))
        return motors

    def start_motors_monitor(self, robot_name):
        self.robot_name = robot_name
        thread = threading.Thread(target=self.motors_monitor)
        thread.daemon = True
        thread.start()

    def motors_monitor(self):
        cmd_dyn = 'rostopic echo /{}/safe/motor_states/default -n 1'.format(self.robot_name)
        while True:
            # Dynamixel states
            try:
                out = EasyProcess(cmd_dyn).call(timeout=1).stdout
                out = out[:out.rfind('\n')]
                states = yaml.load(out)
                self.dynamixel_motors_states = {m['id']: m for m in states['motor_states']}
            except:
                self.dynamixel_motors_states= {}
            # Pololu states
            for i in self.pololu_boards.keys():
                cmd_pol = 'rostopic echo /{}/{}/motors_states -n 1'.format(self.robot_name, i)
                try:
                    out = EasyProcess(cmd_pol).call(timeout=1).stdout
                    out = out[:out.rfind('\n')]
                    states = yaml.load(out)
                    states = dict(zip(states['name'], states['position']))
                    print states
                    self.pololu_boards[i] = states
                except:
                    self.pololu_boards[i] = {}
            time.sleep(1)

    @staticmethod
    def check(cmd, env=None):
        """ Checks a single command for success """
        errcode = call(cmd, stdout=DEVNULL, env=env, shell=True)
        return errcode == 0

    def system_status(self):
        # Template to return
        # Statuses: 0 - OK, 1 - Error, 2 - N/A
        status = {
            'system': {
                'cpu': int(psutil.cpu_percent()),
                'mem': int(psutil.virtual_memory().percent),
                'total_mem': round(psutil.virtual_memory().total/float(1024*1024*1024)),
                'fps': self.get_blender_fps(),
            },
            'robot': {
                'current_name': '',
                'robots': ['sophia'],
            },
            'status': {
                'ros': self.get_ros_status(),
                'blender': self.get_blender_status(),
                'internet': self.get_internet_status(),
                'pololu': self.check("test -e /dev/ttyACM0") * -1 +1,
                'usb2dynamixel': self.check("test -e /dev/ttyUSB0") * -1 +1,
                'camera': self.check("test -e /dev/video0") * -1 +1,
            },
            # Ros nodes based on config
            'nodes': [
                {'name': 'pololu', 'status':0},
                {'name': 'dynamixel', 'status':0}
            ]

        }
        robot_name = self.get_robot_name()
        # Check additional parameters only if robot is started
        if robot_name:
            status["robot"]['current_name'] = robot_name


        return status

    def get_robot_name(self):
        return self.get_ros_param("/robot_name")

    def get_blender_status(self):
        return EasyProcess("pgrep blender").call().return_code

    def get_ros_status(self):
        return EasyProcess("pgrep roscore").call().return_code

    def get_internet_status(self):
        return EasyProcess("ping 8.8.8.8 -c 1 -W 1").call().return_code

    def _build_env(self):
        return {name: check_output(cmd, shell=True)
                for name, cmd in self.config['setup']['env'].items()}

    def get_ros_param(self, param):
        cmd = 'rosparam get {}'.format(param)
        out = EasyProcess(cmd).call()
        if out.return_code > 0:
            return False
        return out.stdout

    def call_ros_service(self, service, args=""):
        cmd = 'rosservice call {} {}'.format(service, args)
        out = EasyProcess(cmd).call()
        if out.return_code > 0:
            return False
        return out.stdout

    def get_blender_fps(self):
        v = self.call_ros_service("/blender_api/get_param ", '"bpy.data.scenes[\'Scene\'].evaFPS"')
        if v == False:
            return 0
        try:
            s = yaml.load(v)
            fps = s['value']
            return fps
        except:
            return 0
        return int(float(v))


def deepupdate(original, new):
    """Updates missing or None values in all 'directly' nested dicts."""
    for key in new.keys():
        if not key in original or original[key] == None:
            original[key] = new[key]
        elif isinstance(original[key], dict) and isinstance(new[key], dict):
            deepupdate(original[key], new[key])
