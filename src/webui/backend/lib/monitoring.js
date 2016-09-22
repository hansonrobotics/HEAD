let os = require('os'),
    yaml = require('js-yaml'),
    yamlIO = require('./yaml'),
    memory = require('./memory'),
    _ = require('lodash'),
    cp = require('child_process'),
    path = require('path');

module.exports = {
    system_status: function (config_dir, robot_name, started) {
        if (typeof started == 'undefined')
            started = -1;

        // Check if software started only if its status wasnt changed on same call,
        // in which case current status would be passed as argument
        if (started < 0)
            started = this.robot_started(robot_name);

        let totalMem = memory.totalMemory(),
            usedMem = memory.usedMemory();

        return {
            software: started,
            system: {
                'cpu': parseInt(os.loadavg()[0] / os.cpus().length * 100),
                'mem': parseInt(usedMem / totalMem * 100),
                'total_mem': parseInt(totalMem),
                'fps': this.get_blender_fps()
            },
            robot: {
                'current_name': robot_name,
                'robots': ['sophia', 'sophia_body'],
            },
            status: {
                internet: this.get_internet_status(),
                ros: this.get_ros_status(),
                blender: this.get_blender_status(),
                pololu: this.check('test', ['-e', '/dev/ttyACM0']) * -1 + 1,
                usb2dynamixel: this.check('test', ['-e', '/dev/ttyUSB0']) * -1 + 1,
                camera: this.check('test', ['-e', '/dev/video0']) * -1 + 1
            },
            motors: this.getMotorStatus(config_dir, robot_name, started),
            nodes: this.getNodeStatus(config_dir, robot_name),
        };
    },
    getNodeStatus: function (config_dir, robot_name) {
        var running = [],
            res = cp.spawnSync('rosnode', ['list'], {encoding: 'utf8'}),
            nodes = yamlIO.readFile(path.join(config_dir, robot_name, 'nodes.yaml'));

        nodes = 'nodes' in nodes ? nodes['nodes'] : [];

        if (res.status === 0)
            running = _.compact(res.stdout.split('\n'));

        for (node of nodes)
            node['status'] = _.includes(running, node['node']) ? 0 : 1;

        return nodes;
    },
    getMotorStatus: function (config_dir, robot_name, started) {
        var checks = [];
        // Check Hardware only if software not running to avoid accessing same hardware
        if (started == 0) {
            let hw = yaml.load(path.join(config_dir, robot_name, 'hw_monitor.yaml'));

            if ('dynamixel' in hw) {
                for (let c of hw.dynamixel) {
                    // Check device
                    let dev = this.check("test", ['-e', c['device']]) * -1 + 1;
                    checks.push({
                        'label': c['label'] + " USB",
                        'status': dev,
                    });

                    // Check for motors if device found
                    let found = 0;
                    if (dev == 0)
                        found = this.get_dynamixel_motor_number(c['device']);

                    let total = 0;
                    if ('motor_count' in c)
                        total = c['motor_count'];
                    // All motors found
                    let st = 1;
                    if (found >= total && total > 0)
                        st = 0;
                    else if (found > 0)
                        st = 2;

                    let val = found.toString();
                    if (total > 0)
                        val += "/" + total;

                    checks.push({
                        'label': c['label'] + " Motors found",
                        'status': st,
                        'value': val,
                    });
                }
            }

            if ('pololu' in hw) {
                for (let c of hw.pololu) {
                    let dev = this.check("test", ['-e', c['device']]) * -1 + 1;
                    checks.push({
                        'label': c['label'] + " USB",
                        'status': dev
                    });

                    let val = 2;
                    if ('channel' in c)
                        val = this.get_pololu_power(c['device'], c['channel']);

                    checks.push({
                        'label': c['label'] + " Power",
                        'status': val
                    });
                }
            }
        }

        checks.push({
            'label': "Internet Connection",
            'status': this.get_internet_status(),
        });

        return checks;
    },
    call_ros_service: function (service, args) {
        if (args.constructor !== Array)
            args = [];

        let res = cp.spawnSync('rosservice', ['call', service].concat(args), {encoding: 'utf8'});

        if (res.status > 0)
            return null;
        else
            return res.stdout;
    },
    get_blender_fps: function () {
        let res = this.call_ros_service("/blender_api/get_param", ['"bpy.data.scenes[\'Scene\'].evaFPS"'])
        if (res) {
            res = yaml.load(res);
            if ('value' in res)
                return res['value'];
        }

        return 0;
    },
    robot_started: function (name) {
        let res = cp.spawnSync('tmux', ['list-sessions'], {encoding: 'utf8'});
        if (res.status <= 0) {
            let lines = res.stdout.split("\n");
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
    get_pololu_power: function (device, channel) {
        return 0;
    },
    get_internet_status: function () {
        return cp.spawnSync('ping', ['8.8.8.8', '-c', '1', '-W', '1'], {encoding: 'utf8'}).status
    },
    get_ros_status: function () {
        return cp.spawnSync('pgrep', ['rosmaster'], {encoding: 'utf8'}).status
    },
    get_blender_status: function () {
        return cp.spawnSync('pgrep', ['blender'], {encoding: 'utf8'}).status;
    },
    check: function (cmd, args, env) {
        return cp.spawnSync(cmd, args, {encoding: 'utf8'}).status === 0;
    },
    get_dynamixel_motor_number: function (device) {
        if (this.mx_tool) {
            let res = cp.spawnSync(this.mx_tool, ['--device', device], {encoding: 'utf8'});
            if (res.status <= 0) return res.stdout.split("\n").length - 1;
        }
        return 0;
    }
};
