os = require('os')
memory = require('./memory')

module.exports =
  startStatus: (config_dir, robot_name, started=-1) ->
    # Check if software started only if its status wasnt changed on same call,
    # in which case current status would be passed as argument
    #if started < 0
    #  started = self.robot_started(robot_name)

    totalMem = memory.totalMemory()
    usedMem = memory.usedMemory()

    status =
      'software': started
      'system':
        'cpu': parseInt(os.loadavg()[0] / os.cpus().length * 100)
        'mem': parseInt(usedMem / totalMem * 100)
        'total_mem': parseInt(totalMem)
      #'status':
        #'internet': self.get_internet_status()
        #'camera': self.check("test -e /dev/video0") * -1 + 1
      'checks': []

    status
