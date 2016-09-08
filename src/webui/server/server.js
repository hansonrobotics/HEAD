var express = require('express'),
  app = express(),
  config = require('./config'),
  monitoring = require('./lib/monitoring'),
  path = require('path'),
  yaml = require('yamljs');

app.use('/public', express.static('../public'));

app.get('/', function (req, res) {
  res.sendFile(path.resolve(__dirname + '/../public/index.html'));
});

app.get('/robot_config.js', function (req, res) {
    var filename = path.resolve(config.configRoot, config.robot, 'webstart.js');
    if (fs.existsSync(filename))
        res.sendFile(filename);
    else
        res.send('module.exports = {};')
});

app.get('/robot_config_schema', function (req, res) {
    res.json(yaml.load(path.join(config.configRoot, 'config_schema.yaml')));
});

app.get('/robot_config/:name', function (req, res) {
    res.json(yaml.load(path.join(config.configRoot, req.params['name'], 'config.yaml')));
});

app.get('/monitor/start_status', function (req, res) {
    res.json(monitoring.startStatus(config.configRoot, config.robot));
});

app.listen(8003);
