let express = require('express'),
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
    shared_performances_folder = 'shared',
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
    let filename = path.resolve(argv.config, argv.robot, 'webstart.js');
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
    // Do not overwrite config
    res.json(req.body);
//    if (yamlIO.writeFile(path.join(argv.config, req.params['name'], 'config.yaml'), req.body))
//        res.json(req.body);
//    else
//        res.sendStatus(500);
});

app.get('/monitor/status', function (req, res) {
    res.json(monitoring.system_status(argv.config, argv.robot));
});

app.get('/motors/get/:name', function (req, res) {
    res.json({motors: yamlIO.readFile(path.join(argv.config, req.params['name'], 'motors_settings.yaml')) || []});
});

app.post('/motors/update/:name', function (req, res) {
    let robot_name = req.params['name'];
    yamlIO.writeFile(path.join(argv.config, robot_name, 'motors_settings.yaml'), req.body);
    ros.updateMotors(robot_name, req.body);
    res.json({error: false});
});

app.get('/expressions/:name', function (req, res) {
    res.json(yamlIO.readFile(path.join(argv.config, req.params['name'], 'expressions.yaml')) || {});
});

app.post('/expressions/update/:name', function (req, res) {
    let robot_name = req.params['name'];
    let save = {error: !yamlIO.writeFile(path.join(argv.config, req.params['name'], 'expressions.yaml'), req.body)}
    ros.updateExpressions(robot_name);
    res.json(save);
});

app.get('/attention_regions/:name', function (req, res) {
    let regions = yamlIO.readFile(path.join(argv.config, req.params['name'], 'attention_regions.yaml') || []);
    // TODO: remove when configs are updated
    if ('attention_regions' in regions) regions = regions['attention_regions'];
    res.json(regions);
});

app.post('/attention_regions/:name', function (req, res) {
    res.json({error: !yamlIO.writeFile(path.join(argv.config, req.params['name'], 'attention_regions.yaml'), req.body)});
});

app.post('/animations/update/:name', function (req, res) {
    let robot_name = req.params['name'];
    let save = {error: !yamlIO.writeFile(path.join(argv.config, req.params['name'], 'animations.yaml'), req.body)}
    ros.updateExpressions(robot_name)
    res.json(save);
});

app.get('/performances/:name', function (req, res) {
    res.json(performances.all([path.join(argv.config, req.params['name'], 'performances'),
        path.join(argv.config, 'common', 'performances')], {skip_nodes: true}));
});

let updatePerformance = function (req, res) {
    let name = req.params['id'].indexOf(shared_performances_folder) === 0 ? 'common' : req.params['name'],
        root = path.join(argv.config, name, 'performances');

    if (performances.update(root, req.params['id'], _.cloneDeep(req.body)))
        res.json(req.body);
    else
        res.sendStatus(500);
};

app.route('/performances/:name/:id*').post(updatePerformance).put(updatePerformance)
    .get(function (req, res) {
        let name = req.params['id'].indexOf(shared_performances_folder) === 0 ? 'common' : req.params['name'],
            dir = path.join(argv.config, name, 'performances');

        res.json(performances.get(dir, req.params['id']));
    });

app.delete('/performances/:name/:id*', function (req, res) {
    let id = path.join(req.param('id') || '', req.params['0'] || ''),
        name = req.params['id'].indexOf(shared_performances_folder) === 0 ? 'common' : req.params['name'];

    res.json(performances.remove(path.join(argv.config, name, 'performances'), req.params['id']));
});

app.post('/run_performance', function (req, res) {
    res.json(performances.run('idf/' + req.body['key']));
});

app.get('/slide_performance/:show/:performance', function (req, res) {
    res.json({result: performances.run(req.params['show'] + "/" + req.params['performance'])});
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

let readPerformanceProperties = function (dir) {
        dir = dir || '.';
        let name = dir.indexOf(shared_performances_folder) === 0 ? 'common' : argv.robot;
        return yamlIO.readFile(path.join(argv.config, name, 'performances', dir, '.properties')) || {};
    },
    writePerformanceProperties = function (dir, data) {
        dir = dir || '.';
        let name = dir.indexOf(shared_performances_folder) === 0 ? 'common' : argv.robot,
            filename = path.join(argv.config, name, 'performances', dir, '.properties'),
            properties = yamlIO.readFile(filename),
            success = yamlIO.writeFile(filename, _.extend(properties, data));

        ros.performances.reloadProperties();
        return success;
    };

app.get('/performance/settings/:path(*)?', function (req, res) {
    res.json(readPerformanceProperties(req.params['path']) || {});
});

app.post('/performance/settings/:path(*)?', function (req, res) {
    res.json(writePerformanceProperties(req.params['path'], req.body || {}));
});

app.get('/keywords/:path(*)?', function (req, res) {
    res.json({keywords: readPerformanceProperties(req.params['path'])['keywords'] || []});
});

app.post('/keywords/:path(*)?', function (req, res) {
    let keywords = _.map(req.body['keywords'] || [], function (k) {
        return k.trim();
    });

    res.json(writePerformanceProperties(req.params['path'], {keywords: keywords}));
});

app.get('/performance/attention/:path(*)?', function (req, res) {
    res.json(readPerformanceProperties(req.params['path'])['regions'] || []);
});

app.post('/performance/attention/:path(*)?', function (req, res) {
    res.json(writePerformanceProperties(req.params['path'], {regions: req.body}));
});

app.get('/performance/variables/:path(*)?', function (req, res) {
    res.json(readPerformanceProperties(req.params['path'])['variables'] || {});
});

app.post('/performance/variables/:path(*)?', function (req, res) {
    res.json(writePerformanceProperties(req.params['path'], {variables: req.body}));
});

if (argv.ssl) {
    let privateKey = fs.readFileSync(path.join(__dirname, 'ssl', 'key.pem')),
        certificate = fs.readFileSync(path.join(__dirname, 'ssl', 'cert.crt'));

    https.createServer({
        key: privateKey,
        cert: certificate
    }, app).listen(argv.port);
} else {
    app.listen(argv.port);
}
