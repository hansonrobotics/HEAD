import yaml
import json
from subprocess import Popen, PIPE, call, check_output, DEVNULL

class Reporter:
    """
    Executes a number of shell commands, interprets their return codes as
    sucess or failure and returns results.
    """

    def __init__(self, yamlfilename):
        self.filename = yamlfilename
        self.load_config()

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
        env = self._build_env()
        # Build commands to prepend to checks
        cmdlist = self.config['setup']['prepend']

        # Do the checks
        statuslist = []
        for check in self.config['checks']:
            cmd = '; '.join(cmdlist + [check['cmd']])
            status = self.check(cmd, env=env)
            statuslist.append(status)

        # Return a copy of self.config['checks'] with a new parameter 'success'
        return [dict(list(check.items()) + [('success', success)])
                for check, success in zip(self.config['checks'], statuslist)]

    def get_motor_topic_names(self):
        # ros_pololu_servo/command
        cmd = ["rosservice", "call", "rosapi/topics_for_type",  "\"type: 'ros_pololu_servo/command'\""]

        p = Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=PIPE)
        output, err = p.communicate()
        status = p.returncode

        output = output.strip().decode('utf-8').replace("topics", '"topics"')
        output = "{" + output + "}"

        if status == 0:
            return json.loads(output)
        else:
            return {"topics": [], "error": err}

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
