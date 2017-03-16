let glob = require('glob'),
    path = require('path'),
    fs = require('fs'),
    yaml = require('js-yaml'),
    yamlIO = require('./yaml'),
    cp = require('child_process'),
    _ = require('lodash'),
    rimraf = require('rimraf'),
    mkdirp = require('mkdirp'),
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
                let id = filename.replace(dir, '').replace(/^[\/]+/, '').replace(/[\/]+$/, '').replace(self.ext, ''),
                    timeline = self.get(dir, id, options)

                if (timeline) {
                    performance['timelines'].push(timeline)
                }
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
        let self = this,
            timelines = []

        if ('timelines' in performance) {
            timelines = performance['timelines']
            // if ('previous_id' in performance)
            //     mv(path.join(dir, performance['previous_id']), path.join(dir, id))
            mkdirp.sync(path.join(dir, id))
        } else {
            timelines = [performance]
        }

        _.each(timelines, function(timeline, i) {
            let current

            if ('previous_id' in timeline) {
                current = self.get(dir, timeline['previous_id'])
                self.remove(dir, timeline['previous_id'])
                delete timeline['previous_id']
            }

            if (!current) current = self.get(dir, timeline['id'])

            if (current && 'ignore_nodes' in timeline && timeline['ignore_nodes'] && 'nodes' in current) {
                timeline['nodes'] = current['nodes']
                delete timeline['ignore_nodes']
            } else
                for (let node of timeline['nodes'])
                    delete node['id'];

            delete timeline['path']
            timelines[i] = timeline
        })

        _.each(timelines, function(timeline) {
            let id = timeline['id']
            delete timeline['id']
            yamlIO.writeFile(path.join(dir, id + self.ext), timeline)
        })

        return this.get(dir, id)
    },
    remove: function(dir, id) {
        let p = path.join(dir, id)
        if (fs.existsSync(p) && fs.lstatSync(p).isDirectory()) {
            rimraf.sync(p, {}, function() {})
        } else
            try {
                fs.unlinkSync(p + this.ext)
            } catch (e) {
                console.log(e)
            }
        return true
    },
    run: function(id) {
        // Async call to make the syncing with multiple PCs easier.
        cp.spawn('rosservice', ['call', '/performances/run_full_performance', id], {encoding: 'utf8'})
        return true
    }
}