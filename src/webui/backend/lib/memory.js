var fs = require('fs');

module.exports = {
    _ramStats: function () {
        return fs.readFileSync('/proc/meminfo', 'utf8');
    },
    _ramField: function (stats, field) {
        var a = stats.indexOf(field),
            b = stats.indexOf('kB', a);

        return parseInt(stats.substring(a + field.length + 1, b).trim(), 10);
    },
    totalMemory: function () {
        return this._ramField(this._ramStats(), 'MemTotal') / 1024;
    },
    usedMemory: function () {
        var stats = this._ramStats(),
            available = this._ramField(stats, 'MemFree') + this._ramField(stats, 'Buffers') + this._ramField(stats, 'Cached');
        return Math.round((this._ramField(stats, 'MemTotal') - available) / 1024);
    }
};