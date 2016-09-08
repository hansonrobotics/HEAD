var os = require('os'),
    memory = require('./memory'),
    cp = require('child_process');

module.exports = {
    startStatus: function (config_dir, robot_name, started) {
        if (typeof started == 'undefined')
            started = -1;

        // Check if software started only if its status wasnt changed on same call,
        // in which case current status would be passed as argument
        //if started < 0
        //  started = self.robot_started(robot_name)

        var totalMem = memory.totalMemory(),
            usedMem = memory.usedMemory(),
            status = {
                'software': started,
                'system': {
                    'cpu': parseInt(os.loadavg()[0] / os.cpus().length * 100),
                    'mem': parseInt(usedMem / totalMem * 100),
                    'total_mem': parseInt(totalMem)
                },
                //'status':
                //  'internet': self.get_internet_status()
                //  'camera': self.check("test -e /dev/video0") * -1 + 1
                'checks': []
            };
        console.log(this.get_blender_fps());
        return status;
    },
    call_ros_service: function (service, args) {
        if (args.constructor !== Array)
            args = [];

        return cp.spawnSync('rosservice', ['call', service].concat(args), { encoding : 'utf8' });
    },
    get_blender_fps: function () {
        return this.call_ros_service("/blender_api/get_param", ['"bpy.data.scenes[\'Scene\'].evaFPS"'])
    }
};
