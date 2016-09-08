var os = require('os'),
    yaml = require('yamljs'),
    memory = require('./memory'),
    cp = require('child_process'),
    path = require('path');

module.exports = {
    startStatus: function (config_dir, robot_name, started) {
        if (typeof started == 'undefined')
            started = -1;

        // Check if software started only if its status wasnt changed on same call,
        // in which case current status would be passed as argument
        if (started < 0)
            started = this.robot_started(robot_name);

        var totalMem = memory.totalMemory(),
            usedMem = memory.usedMemory(),
            status = {
                'software': started,
                'system': {
                    'cpu': parseInt(os.loadavg()[0] / os.cpus().length * 100),
                    'mem': parseInt(usedMem / totalMem * 100),
                    'total_mem': parseInt(totalMem),
                    'fps': this.get_blender_fps()
                },
                'status': {
                    'internet': this.get_internet_status(),
                    'ros': this.get_ros_status(),
                    'blender': this.get_blender_status(),
                    'pololu': this.check('test', ['-e', '/dev/ttyACM0']) * -1 + 1,
                    'usb2dynamixel': this.check('test', ['-e', '/dev/ttyUSB0']) * -1 + 1,
                    'camera': this.check('test', ['-e', '/dev/video0']) * -1 + 1
                },
                'checks': []
            };

        if (status['software'] == 0) {
            var hw = yaml.load(path.join(config_dir, robot_name, 'hw_monitor.yaml'));
            if ('dynamixel' in hw) {
                for (let c of hw.dynamixel) {
                    // Check device
                    var dev = this.check("test", ['-e', c['device']]) * -1 + 1;
                    status['checks'].push({
                        'label': c['label'] + " USB",
                        'status': dev,
                    });

                    // Check for motors if device found
                    var found = 0;
                    if (dev == 0)
                        found = this.get_dynamixel_motor_number(c['device'])

                    var total = 0;
                    if ('motor_count' in c)
                        total = c['motor_count'];
                    // All motors found
                    var st = 1;
                    if (found >= total && total > 0)
                        st = 0;
                    else if (found > 0)
                        st = 2;

                    var val = str(found);
                    if (total > 0)
                        val += "/{}".format(total);

                    status['checks'].append({
                        'label': c['label'] + " Motors found",
                        'status': st,
                        'value': val,
                    });
                }
            }
        }

        status['checks'].push({
            'label': "Internet Connection",
            'status': this.get_internet_status(),
        });

        return status;
    },
    call_ros_service: function (service, args) {
        if (args.constructor !== Array)
            args = [];

        var res = cp.spawnSync('rosservice', ['call', service].concat(args), {encoding: 'utf8'});

        if (res.status > 0)
            return null;
        else
            return res.stdout;
    },
    get_blender_fps: function () {
        var res = this.call_ros_service("/blender_api/get_param", ['"bpy.data.scenes[\'Scene\'].evaFPS"'])
        if (res) {
            res = yaml.parse(res);
            if ('value' in res)
                return res['value'];
        }

        return 0;
    },
    robot_started: function (name) {
        var res = cp.spawnSync('tmux', ['list-sessions'], {encoding: 'utf8'});
        if (res.status <= 0) {
            var lines = res.stdout.split("\n");
            for (let line of lines) {
                if (line.indexOf(name) !== -1) {
                    if (line.indexOf('attached') !== -1)
                        return 1;
                    else
                        return 2;
                }
            }
        }
        return 0;
    },
    get_internet_status: function () {
        return cp.spawnSync('ping', ['8.8.8.8', '-c', '1', '-W', '1'], {encoding: 'utf8'}).status
    },
    get_ros_status: function () {
        return cp.spawnSync('pgrep', ['roscore'], {encoding: 'utf8'}).status
    },
    get_blender_status: function () {
        return cp.spawnSync('pgrep', ['blender'], {encoding: 'utf8'}).status;
    },
    check: function (cmd, args) {
        return cp.spawnSync(cmd, args, {encoding: 'utf8'}).status === 0;
    },
    get_dynamixel_motor_number: function(device) {
        if (self.mx_tool) {
            var res = cp.spawnSync(self.mx_tool, ['--device', device], {encoding: 'utf8'});
            if (res.status <= 0) return res.stdout.split("\n").length - 1;
        }
        return 0;
    }
};
