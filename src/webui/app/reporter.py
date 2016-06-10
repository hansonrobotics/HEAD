import yaml
from subprocess import call, check_output, Popen
from easyprocess import EasyProcess
import threading
import time
from pololu.motors import Maestro
from configs import Configs
import math
import psutil
import time

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
        self.mx_tool = False
        self.exe = None
        self.exe_stop = None

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
                    self.pololu_boards[i] = states
                except:
                    self.pololu_boards[i] = {}
            time.sleep(1)

    @staticmethod
    def check(cmd, env=None):
        """ Checks a single command for success """
        errcode = call(cmd, stdout=DEVNULL, env=env, shell=True)
        return errcode == 0

    def system_status(self, config_dir):
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
            'nodes': []

        }
        robot_name = self.get_robot_name()
        # Check additional parameters only if robot is started
        if robot_name:
            status["robot"]['current_name'] = robot_name
            # ROS Nodes
            nodes_running = str(EasyProcess("rosnode list").call().stdout).splitlines()
            nod_file = os.path.join(config_dir,robot_name,'nodes.yaml')
            with open(nod_file, 'r') as stream:
                node_cfg = yaml.load(stream)
            for i,n in enumerate(node_cfg['nodes']):
                if len([nd for nd in nodes_running if nd == n['node']]) > 0:
                    node_cfg['nodes'][i]['status'] = 0
                else:
                    node_cfg['nodes'][i]['status'] = 1

            status['nodes'] = node_cfg['nodes']

        return status

    def start_status(self, config_dir, robot_name, started = -1):
        # Check if software started only if its status wasnt changed on same call,
        # in which case current status would be passed as argument
        if started < 0:
            started = self.robot_started(robot_name)

        status = {
            'software': started,
            'system': {
                'cpu': int(psutil.cpu_percent()),
                'mem': int(psutil.virtual_memory().percent),
                'total_mem': round(psutil.virtual_memory().total/float(1024*1024*1024)),
            },
            'status': {
               'internet': self.get_internet_status(),
               'camera': self.check("test -e /dev/video0") * -1 +1,
            },
            # Hardware Checks
            'checks': []
        }
        # Check Hardware only if software not running to avoid accessing same hardware
        if robot_name and status['software'] == 0:
            # Check if software already running:

            # Hardware checks
            checks = os.path.join(config_dir,robot_name,'hw_monitor.yaml')
            with open(checks, 'r') as stream:
                hw = yaml.load(stream)
            # Check dynamixels
            if 'dynamixel' in hw:
                for c in hw['dynamixel']:
                    # Check device
                    dev =  self.check("test -e "+c['device']) * -1 +1
                    status['checks'].append({
                        'label': c['label'] + " USB",
                        'status': dev,
                    })
                    # Check for motors if device found
                    if dev == 0:
                        found = self.get_dynamixel_motor_number(c['device'])
                    else:
                        found = 0
                    if 'motor_count' in c:
                        total = c['motor_count']
                    else:
                        total = 0
                    # All motors found
                    st = 1
                    if found >= total > 0:
                        st = 0
                    elif found > 0:
                        st = 2
                    val = str(found)
                    if total > 0:
                        val += "/{}".format(total)

                    status['checks'].append({
                        'label': c['label'] + " Motors found",
                        'status': st,
                        'value': val,
                    })
            if 'pololu' in hw:
                for c in hw['pololu']:
                    dev =  self.check("test -e "+c['device']) * -1 +1
                    status['checks'].append({
                        'label': c['label'] + " USB",
                        'status': dev,
                    })
                    val = 2
                    if 'channel' in c:
                        val = self.get_pololu_power(c['device'], c['channel'])
                    status['checks'].append({
                        'label': c['label'] + " Power",
                        'status': val,
                    })
        status['checks'].append({
            'label': "Internet Connection",
            'status': self.get_internet_status(),
        })
        return status
    def get_dynamixel_motor_number(self, device):
        if not self.mx_tool:
            return 0
        cmd = "{} --device {}".format(self.mx_tool, device)
        out = EasyProcess(cmd).call()
        if out.return_code > 0:
            return 0
        return len(out.stdout.splitlines())-1

    def get_pololu_power(self):
        return 0

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

    # Starts robot script in new terminal (gnome by default)
    def start_robot(self, robot_name):
        if not self.exe:
            return False
        cmd = 'gnome-terminal -e "{} {}"'.format(self.exe, robot_name)
        Popen(cmd, shell=True, stdin=None, stdout=None, stderr=None, close_fds=True)
        return True

    def stop_robot(self, robot_name):
        if not self.exe_stop:
            return False
        cmd = 'gnome-terminal -e "{}"'.format(self.exe_stop)
        out = EasyProcess(cmd).call()
        if out.return_code > 0:
            return False
        return True

    # Returns 0 robot has not been started, 1 - if its is started, 2 - if it is still starting up.
    # Based on TMUX session list
    def robot_started(self, robot_name):
        if not robot_name:
            return 0
        cmd ='tmux list-sessions'
        out = EasyProcess(cmd).call()
        if out.return_code > 0:
            return 0
        sessions = str(EasyProcess(cmd).call().stdout).splitlines()
        for s in sessions:
            if s.find(robot_name) > -1:
                if s.find('attached') > -1:
                    return 1
                else:
                    return 2
        return 0

    def get_pololu_power(self, port, channel):
        try:
            m = Maestro(port);
            pos = m.getPosition(channel)
            m.close()
            if pos > 100:
                return 0
            return 1
        except:
            return 1


def deepupdate(original, new):
    """Updates missing or None values in all 'directly' nested dicts."""
    for key in new.keys():
        if not key in original or original[key] == None:
            original[key] = new[key]
        elif isinstance(original[key], dict) and isinstance(new[key], dict):
            deepupdate(original[key], new[key])
