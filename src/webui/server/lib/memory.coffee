fs = require('fs')

module.exports =
  _ramStats: ->
    fs.readFileSync('/proc/meminfo', 'utf8')
  _ramField: (stats, field) ->
    a = stats.indexOf(field)
    b = stats.indexOf('kB', a);
    parseInt(stats.substring(a + field.length + 1, b).trim(), 10)
  totalMemory: ->
    this._ramField(this._ramStats(), 'MemTotal') / 1024
  usedMemory: ->
    stats = this._ramStats();
    available = this._ramField(stats, 'MemFree') + this._ramField(stats, 'Buffers') + this._ramField(stats, 'Cached');
    Math.round((this._ramField(stats, 'MemTotal') - available) / 1024);
