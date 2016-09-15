var cp = require('child_process');

module.exports = {
    publish: function (topic, type, data) {
        var res = cp.spawnSync('rostopic', ['pub', '--once', topic, type, JSON.stringify(data)]);
        return res.status === 0;
    }
};
