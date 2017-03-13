let glob = require('glob'),
    path = require('path'),
    fs = require('fs'),
    yaml = require('js-yaml'),
    yamlIO = require('./yaml'),
    cp = require('child_process'),
    _ = require('lodash'),
    rimraf = require('rimraf'),
    mv = require('mv')

module.exports = {
    ext: '.yaml',
    all: function(dirs, options) {
        options = typeof options === 'object' ? options : {}

        if (dirs.constructor !== Array)
            dirs = [dirs]

        let performances = []

        for (let dir of dirs) {
            let subDirs = glob.sync(path.join(dir, '**', '*/'))

            for (let i = 0; i < subDirs.length; i++) {
                // filter out dirs containing other dirs on a sorted arrray
                if (i === subDirs.length - 1 || subDirs[i + 1].indexOf(subDirs[i]) === -1) {
                    let id = subDirs[i].replace(dir, '').replace(/^[\/]+/, '').replace(/[\/]+$/, ''),
                        performance = this.get(dir, id, options)

                    performances.push(performance)
                }
            }
        }

        return performances
    },
    get: function(dir, id, options) {
        options = typeof options === 'object' ? options : {}
        let self = this,
            performance = null,
            p = path.join(dir, id)

        if (fs.existsSync(p) && fs.lstatSync(p).isDirectory()) {
            let performancePath = path.dirname(id),
                files = glob.sync(path.join(p, '*.yaml'))

            performancePath = performancePath === '.' ? '' : performancePath
            performance = {id: id, name: path.basename(id), path: performancePath, timelines: []}

            files.forEach(function(filename) {
                filename = p + filename
                fs.readFile(filename, 'utf-8', function(err, content) {
                    let id = filename.replace(dir, '').replace(/^[\/]+/, '').replace(/[\/]+$/, '')
                    if (!err) performance['timelines'].push(self.parseTimeline(id, content, options))
                })
            })
        } else {
            performance = yamlIO.readFile(p + this.ext)

            if (performance)
                performance = this.parseTimeline(id, performance, options)
        }


        return performance
    },
    parseTimeline: function(id, performance, options) {
        performance['id'] = id
        performance['path'] = path.dirname(id)
        performance['path'] = performance['path'] === '.' ? '' : performance['path']
        performance['name'] = path.basename(id)

        if ('skip_nodes' in options && options['skip_nodes'])
            delete performance['nodes']

        return performance
    },
    update: function(dir, id, performance) {
        let self = this

        if ('timelines' in performance) {
            _.each(performance['timelines'], function(timeline) {
                self.update(dir, timeline['id'], timeline)
            })

            if ('previous_id' in performance) {
                mv(path.join(dir, performance['previous_id']), path.join(dir, id))
            }

            return true
        } else {
            let current

            if ('previous_id' in performance) {
                console.log(performance['previous_id'])
                console.log(performance['id'])

                current = this.get(dir, performance['previous_id'])
                this.remove(dir, performance['previous_id'])
                delete performance['previous_id']
            }

            if (!current) current = this.get(dir, id)

            if ('ignore_nodes' in performance && performance['ignore_nodes'] && 'nodes' in current) {
                performance['nodes'] = current['nodes']
                delete performance['ignore_nodes']
            } else
                for (let node of performance['nodes'])
                    delete node['id'];

            delete performance['id']
            delete performance['path']

            yamlIO.writeFile(path.join(dir, id + this.ext), performance)
            return this.get(dir, id)
        }
    },
    remove: function(dir, id) {
        let p = path.join(dir, id)
        if (fs.existsSync(p) && fs.lstatSync(p).isDirectory()) {
            rimraf.sync(p, {}, function() {})
        } else
            fs.unlinkSync(p + this.ext)
        return true
    },
    run: function(id) {
        // Async call to make the syncing with multiple PCs easier.
        cp.spawn('rosservice', ['call', '/performances/run_full_performance', id], {encoding: 'utf8'})
        return true
    }
}