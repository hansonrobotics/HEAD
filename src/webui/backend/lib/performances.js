var glob = require('glob'),
    path = require('path'),
    fs = require('fs'),
    yaml = require('js-yaml'),
    yamlIO = require('./yaml'),
    cp = require('child_process'),
    _ = require('lodash');

module.exports = {
    ext: '.yaml',
    all: function (dirs, options) {
        options = typeof options == 'object' ? options : {};

        if (dirs.constructor !== Array)
            dirs = [dirs];

        var performances = [];

        for (let dir of dirs) {
            var files = glob.sync(path.join(dir, '**', '*.yaml'));

            for (var file of files) {
                var id = file.replace(path.join(dir), '').replace(this.ext, '').substr(1),
                    performance = this.get(dir, id, options);

                if (performance) performances.push(performance);
            }
        }

        return performances;
    },
    get: function (dir, id, options) {
        options = typeof options == 'object' ? options : {};
        var performance = yamlIO.readFile(path.join(dir, id + this.ext));

        if (performance) {
            performance['id'] = id;
            performance['path'] = path.dirname(id);
            performance['path'] = performance['path'] === '.' ? '' : performance['path']
            performance['name'] = path.basename(id);

            if ('skip_nodes' in options && options['skip_nodes'])
                delete performance['nodes'];
        }

        return performance;
    },
    update: function (dir, id, performance) {
        var current;

        if ('previous_id' in performance) {
            current = this.get(dir, performance['previous_id']);
            this.remove(dir, performance['previous_id']);
            delete performance['previous_id'];
        }

        if (!current) current = this.get(dir, id);

        if ('ignore_nodes' in performance && performance['ignore_nodes'] && 'nodes' in current) {
            performance['nodes'] = current['nodes'];
            delete performance['ignore_nodes'];
        } else
            for (let node of performance['nodes'])
                delete node['id'];


        delete performance['id'];
        delete performance['path'];

        return yamlIO.writeFile(path.join(dir, id + this.ext), performance);
    },
    remove: function (dir, id) {
        try {
            fs.unlinkSync(path.join(dir, id + this.ext));
        } catch (e) {
            return false;
        }
        return true;
    },
    run: function (id) {
        var res = cp.spawnSync('rosservice', ['call', '/performances/run_full_performance', id], {encoding: 'utf8'});
        return res.status === 0 && yaml.load(res.stdout)['success'];
    }
};