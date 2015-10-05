import yaml
from subprocess import call, check_output
from easyprocess import EasyProcess
import threading
import time
import re

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
        self.dynamixel_started = False
        self.dynamixel_motors = []
        self.robot_name = ''

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
        if not self.dynamixel_started:
            self.start_dynamixel_monitor(robot_name)
            self.dynamixel_started = True
            # Sleep some time so first results if motors are alive will have time to return
            time.sleep(0.3)
        status = {}
        pololu_boards = {}
        for i, m in enumerate(motors):
            if m['hardware'] == 'pololu':
                if m['topic'] not in pololu_boards.keys():
                    cmd = "rosparam get /{}/{}_enabled".format(robot_name, m['topic'])
                    pololu_boards[m['topic']] = 2 if EasyProcess(cmd).call().stdout == str("true") else 1

                motors[i]['error'] = pololu_boards[m['topic']]
            #Dynamixel motors
            else:
                if m['motor_id'] in self.dynamixel_motors:
                    motors[i]['error'] = 0
                else:
                    # Motor is not on
                    motors[i]['error'] = 1
        return motors

    def start_dynamixel_monitor(self, robot_name):
        self.robot_name = robot_name
        thread = threading.Thread(target=self.dynamixel_monitor)
        thread.daemon = True
        thread.start()

    def dynamixel_monitor(self):
        cmd = 'rostopic echo /{}/safe/motor_states/default -n 1'.format(self.robot_name)
        while True:
            out = EasyProcess(cmd).call(timeout=1).stdout
            self.dynamixel_motors = [int(filter(str.isdigit, str(id))) for id in re.findall(r"id: \d+",out)]
            print self.dynamixel_motors
            time.sleep(1)

    @staticmethod
    def check(cmd, env=None):
        """ Checks a single command for success """
        errcode = call(cmd, stdout=DEVNULL, env=env, shell=True)
        return errcode == 0

    def _build_env(self):
        return {name: check_output(cmd, shell=True)
                for name, cmd in self.config['setup']['env'].items()}


def deepupdate(original, new):
    """Updates missing or None values in all 'directly' nested dicts."""
    for key in new.keys():
        if not key in original or original[key] == None:
            original[key] = new[key]
        elif isinstance(original[key], dict) and isinstance(new[key], dict):
            deepupdate(original[key], new[key])
