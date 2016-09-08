var express = require('express'),
    app = express(),
    config = require('./config'),
    monitoring = require('./lib/monitoring'),
    fs = require('fs'),
    path = require('path'),
    yaml = require('yamljs'),
    argv = require('yargs')
        .usage('Usage: $0 [options]')
        .options({
            r: {
                alias: 'robot',
                default: 'sophia',
                describe: 'Robot name'
            },
            e: {
                alias: 'exec',
                describe: 'Robot execution script'
            },
            s: {
                alias: 'stop',
                describe: 'Robot stop script'
            },
            c: {
                alias: 'config',
                describe: 'Robot config dir'
            },
            p: {
                alias: 'port',
                type: 'number',
                describe: 'Port',
                default: 8003
            },
            m: {
                alias: 'mx',
                describe: 'Dynamixel motor command line tool'
            }
        }).help('h').alias('h', 'help').argv;

app.use('/public', express.static('../public'));

app.get('/', function (req, res) {
    res.sendFile(path.resolve(__dirname + '/../public/index.html'));
});

app.get('/robot_config.js', function (req, res) {
    var filename = path.resolve(config.configRoot, config.robot, 'webstart.js');
    if (monitoring.robot_started(argv.robot) === 1 || !fs.existsSync(filename)) {
        res.send("define(function (){return {mode:'normal'}});");
    } else {
        res.sendFile(filename);
    }
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

app.listen(argv.port);
