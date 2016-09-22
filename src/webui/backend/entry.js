var express = require('express'),
    https = require("https"),
    app = express(),
    monitoring = require('./lib/monitoring'),
    fs = require('fs'),
    path = require('path'),
    yamlIO = require('./lib/yaml'),
    performances = require('./lib/performances'),
    bodyParser = require('body-parser'),
    mkdirp = require('mkdirp'),
    PythonShell = require('python-shell'),
    ros = require('./lib/ros'),
    _ = require('lodash'),
    argv = require('yargs').options({
        r: {
            alias: 'robot',
            describe: 'Robot name',
            demand: true
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
        s: {
            alias: 'ssl',
            describe: 'Enable ssl',
            type: 'boolean'
        }
    }).usage('Usage: $0 [options]').help('h').alias('h', 'help').argv;

app.use(bodyParser.urlencoded({extended: false}));
app.use(bodyParser.json());
app.use('/public', express.static(path.join(__dirname, '../public')));

app.get('/', function (req, res) {
    res.sendFile(path.resolve(path.join(__dirname, '../public/index.html')));
});

app.get('/robot_config.js', function (req, res) {
    var filename = path.resolve(argv.config, argv.robot, 'webstart.js');
    if (monitoring.robot_started(argv.robot) === 1) {
        res.send("define(function (){return {mode:'normal'}});");
    } else {
        res.sendFile(filename);
    }
});

app.get('/robot_config_schema', function (req, res) {
    res.json(yamlIO.readFile(path.join(argv.config, 'config_schema.yaml')) || {});
});

app.get('/robot_config/:name', function (req, res) {
    res.json(yamlIO.readFile(path.join(argv.config, req.params['name'], 'config.yaml')) || {});
});

app.post('/robot_config/:name', function (req, res) {
    if (yamlIO.writeFile(path.join(argv.config, req.params['name'], 'config.yaml'), req.body))
        res.json(req.body);
    else
        res.sendStatus(500);
});

app.get('/monitor/status', function (req, res) {
    res.json(monitoring.system_status(argv.config, argv.robot));
});

app.get('/motors/get/:name', function (req, res) {
    res.json({motors: yamlIO.readFile(path.join(argv.config, req.params['name'], 'motors_settings.yaml')) || []});
});

app.post('/motors/update/:name', function (req, res) {
    var robot_name = req.params['name'];
    yamlIO.writeFile(path.join(argv.config, robot_name, 'motors_settings.yaml'), req.body);
    ros.updateMotors(robot_name, req.body);
    res.json({error: false});
});

app.get('/expressions/:name', function (req, res) {
    res.json(yamlIO.readFile(path.join(argv.config, req.params['name'], 'expressions.yaml')) || {});
});

app.post('/expressions/update/:name', function (req, res) {
    res.json({error: !yamlIO.writeFile(path.join(argv.config, req.params['name'], 'expressions.yaml'), req.body)});
});

app.get('/attention_regions/:name', function (req, res) {
    var regions = yamlIO.readFile(path.join(argv.config, req.params['name'], 'attention_regions.yaml') || []);
    // TODO: remove when configs are updated
    if ('attention_regions' in regions) regions = regions['attention_regions'];
    res.json(regions);
});

app.post('/attention_regions/:name', function (req, res) {
    res.json({error: !yamlIO.writeFile(path.join(argv.config, req.params['name'], 'attention_regions.yaml'), req.body)});
});

app.post('/animations/update/:name', function (req, res) {
    res.json(yamlIO.writeFile(path.join(argv.config, req.params['name'], 'animations.yaml'), req.body));
});

app.get('/performances/:name', function (req, res) {
    res.json(performances.all(path.join(argv.config, req.params['name'], 'performances'), {skip_nodes: true}));
});

var updatePerformance = function (req, res) {
    var dir = path.join(argv.config, req.params['name'], 'performances');
    if (performances.update(dir, req.params['id'], req.body))
        res.json(performances.get(dir, req.params['id']));
    else
        res.sendStatus(500);
};

app.post('/performances/:name/:id', updatePerformance);
app.put('/performances/:name/:id', updatePerformance);

app.delete('/performances/:name/:id', function (req, res) {
    res.json(performances.remove(path.join(argv.config, req.params['name'], 'performances'), req.params['id']));
});

app.post('/run_performance', function (req, res) {
    res.json(performances.run('idf/' + req.body['key']));
});

app.get('/slide_performance/:performance', function (req, res) {
    res.json({result: performances.run('idf/' + req.params['performance'])});
});

app.post('/lookat', function (req, res) {
    ros.lookAt({
        x: req.body['x'],
        y: -1 * req.body['y'],
        z: req.body['z']
    });

    res.json({result: true});
});

app.post('/monitor/logs/:level', function (req, res) {
    PythonShell.run('scripts/logs.py', {
        args: [req.params['level'], JSON.stringify(req.body)]
    }, function (err, data) {
        if (err)
            res.json([]);
        else
            res.json(JSON.parse(data));
    });
});

var updateKeywords = function (req, res) {
    var keywords = _.compact(req.body['keywords'] || []);
    console.log('save');
    console.log(path.join(argv.config, req.params['name'], 'performances', req.params['path'] || '', '.properties'));

    res.json(yamlIO.writeFile(
        path.join(argv.config, req.params['name'], 'performances', req.params['path'] || '', '.properties'), {keywords: keywords}));
};

app.get('/performances/keywords/:name/:path*?', function (req, res) {
    console.log('get');

    res.json(
        yamlIO.readFile(
            path.join(argv.config, req.params['name'], 'performances', req.params['path'] || '', '.properties')) || {keywords: []});
});

app.post('/performances/keywords/update/:name', updateKeywords);
app.post('/performances/keywords/update/:name/:path*?', updateKeywords);

if (argv.ssl) {
    var privateKey = fs.readFileSync(path.join(__dirname, 'ssl', 'key.pem')),
        certificate = fs.readFileSync(path.join(__dirname, 'ssl', 'cert.crt'));

    https.createServer({
        key: privateKey,
        cert: certificate
    }, app).listen(argv.port);
} else {
    app.listen(argv.port);
}

