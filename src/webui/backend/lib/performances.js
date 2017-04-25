let glob = require('glob'),
    path = require('path'),
    fs = require('fs'),
    yaml = require('js-yaml'),
    yamlIO = require('./yaml'),
    cp = require('child_process'),
    _ = require('lodash'),
    rimraf = require('rimraf'),
    mkdirp = require('mkdirp'),
    mv = require('mv'),
    spawn = require('child_process').spawn

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
        performance['path'] = path.dirname(id).replace('.', '')
        performance['name'] = path.basename(id)

        if ('skip_nodes' in options && options['skip_nodes'])
            delete performance['nodes']

        return performance
    },
    update: function(dir, id, performance) {
        let self = this,
            timelines = [],
            // use this path for all timelines
            p = id

        if (!('timelines' in performance)) {
            timelines = [performance]
            p = path.dirname(performance['id']).replace('.', '')
        } else {
            timelines = 'timelines' in performance ? performance['timelines'] : []
            mkdirp.sync(path.join(dir, id))

            // remove timelines not included in the current performances
            let ids = timelines.map(t => t.id),
                deleteIds = _.pullAll(
                    glob.sync(path.join(dir, id, '*.yaml')).map(f => path.relative(dir, f).replace(this.ext, '')), ids)

            deleteIds.forEach(function(id) {
                self.remove(dir, id)
            })

            if ('previous_id' in performance)
                spawn('mv', ['-T', path.join(dir, performance['previous_id']), path.join(dir, performance['id'])])
        }

        _.each(timelines, function(timeline, i) {
            timeline['id'] = path.join(p, path.basename(timeline['id']))
            if ('previous_id' in timeline)
                timeline['previous_id'] = path.join(p, path.basename(timeline['previous_id']))

            let current,
                ignoreNodes = 'ignore_nodes' in timeline && timeline['ignore_nodes']

            if ('previous_id' in timeline) {
                if (ignoreNodes)
                    current = self.get(dir, timeline['previous_id'])

                // remove previous timeline
                self.remove(dir, timeline['previous_id'])
            }

            if (ignoreNodes && !current) current = self.get(dir, timeline['id'])
            if (current && ignoreNodes && 'nodes' in current) {
                timeline['nodes'] = current['nodes']
            }

            if ('nodes' in timeline)
                for (let node of timeline['nodes'])
                    delete node['id'];

            for (let k of ['path', 'ignore_nodes', 'previous_id'])
                delete timeline[k]

            timelines[i] = timeline
        })

        // save performances separately so that all the obsolete file are already deleted
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
            rimraf.sync(p, {}, function(e) {
                if (e) console.log(e)
            })
        } else
            try {
                fs.unlinkSync(p + this.ext)
            } catch (e) {
                console.log('Error: ', e)
            }
        return true
    },
    run: function(id) {
        // Async call to make the syncing with multiple PCs easier.
        cp.spawn('rosservice', ['call', '/performances/run_full_performance', id], {encoding: 'utf8'})
        return true
    }
}