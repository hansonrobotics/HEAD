var express = require('express'),
    app = express(),
    monitoring = require('./lib/monitoring'),
    fs = require('fs'),
    path = require('path'),
    yaml = require('yamljs'),
    argv = require('yargs').options({
        r: {
            alias: 'robot',
            describe: 'Robot name',
            demand: true
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
            describe: 'Robot config dir',
            demand: true
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
    }).usage('Usage: $0 [options]').help('h').alias('h', 'help').argv;

app.use('/public', express.static('../public'));

app.get('/', function (req, res) {
    res.sendFile(path.resolve(__dirname + '/../public/index.html'));
});

app.get('/robot_config.js', function (req, res) {
    var filename = path.resolve(argv.config, argv.robot, 'webstart.js');
    if (monitoring.robot_started(argv.robot) === 1 || !fs.existsSync(filename)) {
        res.send("define(function (){return {mode:'normal'}});");
    } else {
        res.sendFile(filename);
    }
});

app.get('/robot_config_schema', function (req, res) {
    res.json(yaml.load(path.join(argv.config, 'config_schema.yaml')));
});

app.get('/robot_config/:name', function (req, res) {
    res.json(yaml.load(path.join(argv.config, req.params['name'], 'config.yaml')));
});

app.get('/monitor/start_status', function (req, res) {
    res.json(monitoring.startStatus(argv.config, argv.robot));
});

app.listen(argv.port);
