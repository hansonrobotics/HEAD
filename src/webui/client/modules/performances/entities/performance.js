define(['application', 'backbone', 'lib/api', './node_collection', 'underscore', './node'],
    function(App, Backbone, api, NodeCollection, _) {
        return Backbone.Model.extend({
            urlRoot: function() {
                return '/performances/' + api.config.robot
            },
            initialize: function(options) {
                let self = this

                this.nodes = new NodeCollection()
                this.set('nodes', this.nodes)

                if (options && options.nodes)
                    _.each(options.nodes, function(attributes) {
                        delete attributes['id']
                        self.nodes.add(new self.nodes.model(attributes))
                    })

                this.updateDuration()
                // update duration and trigger change event on nodes' change
                this.nodes.on('reset change update', function() {
                    self.updateDuration()
                    self.trigger('change:duration', self)
                })

                // making sure nodes attribute holds NodeCollection instance
                this.on('change:nodes', function() {
                    if (this.get('nodes') !== this.nodes) {
                        // updating collection
                        this.nodes.reset(this.get('nodes'))
                        this.set({nodes: this.nodes})
                    }
                })

                // update id
                this.on('change:name change:path', this.updateId)
                this.updateId()
            },
            fetchCurrent: function(options) {
                let self = this
                options = options || {}
                api.services.performances.current.callService({}, function(response) {
                    response.performance = JSON.parse(response.performance)
                    self.clear().set(response.performance)

                    if (typeof options.success === 'function')
                        options.success(response)
                }, function(error) {
                    if (typeof options.error === 'function')
                        options.error(error)
                })
            },
            enableSync: function(callback) {
                this.disableSync()
                this.syncCallback = function(msg) {
                    let performance = JSON.parse(msg.data)
                    if (typeof callback === 'function') callback(performance)
                }

                api.topics.running_performance.subscribe(this.syncCallback)
            },
            disableSync: function() {
                if (this.syncCallback) api.topics.running_performance.unsubscribe(this.syncCallback)
            },
            load: function(options) {
                let self = this
                options = options || {}
                api.services.performances.load.callService({
                    id: this.id
                }, function(response) {
                    if (response.success) {
                        response.performance = JSON.parse(response.performance)
                        self.clear({silent: true}).set(response.performance)
                        if (typeof options.success === 'function')
                            options.success(response)
                    } else if (typeof options.error === 'function')
                        options.error('Another performance is running')
                }, function(error) {
                    if (typeof options.error === 'function')
                        options.error(error)
                })
            },
            loadPerformance: function(options) {
                options = options || {}
                api.services.performances.load_performance.callService({
                    performance: JSON.stringify(this.toJSON())
                }, function(response) {
                    if (response.success) {
                        if (typeof options.success === 'function')
                            options.success(response)
                    } else if (typeof options.error === 'function')
                        options.error('Another performance is running')
                }, function(error) {
                    if (typeof options.error === 'function')
                        options.error(error)
                })
            },
            parse: function(response) {
                for (let attr of ['previous_id', 'ignore_nodes']) {
                    delete response[attr]
                    this.unset(attr)
                }

                if ('timelines' in response && this.nodes)
                    this.nodes.reset()

                return response
            },
            updateId: function() {
                if (this.get('name')) {
                    let name = this.get('name').replace(/ /g, '_').replace(/[^\w-]+/g, ''),
                        path = this.get('path') ? this.get('path').split('/') : []

                    path.push(name)

                    this.set('id', path.join('/'))

                    if (!this.get('previous_id') && this.previous('id') && this.previous('id') !== this.get('id'))
                        this.set('previous_id', this.previous('id'))
                }
            },
            run: function(startTime, options) {
                let self = this
                if (!options) options = {}

                api.services.performances.run.callService({
                    startTime: startTime
                }, function(response) {
                    if (response.success) {
                        self.paused = false
                        self.startTime = response.time
                        self.trigger('run', startTime)

                        if (typeof options.success === 'function')
                            options.success(response)
                    } else if (typeof options.error === 'function')
                        options.error('Another performance is running')
                }, function(error) {
                    if (typeof options.error === 'function')
                        options.error(error)
                })
            },
            resume: function(options) {
                let self = this
                if (!options) options = {}

                api.services.performances.resume.callService({}, function(response) {
                    if (response.success) {
                        self.trigger('resume', self.resumeTime)
                        self.paused = false
                        response.time = self.resumeTime
                        self.resumeTime = null
                        if (typeof options.success === 'function')
                            options.success(response)
                    } else if (typeof options.error === 'function')
                        options.error('Unable to resume performance')
                }, function(error) {
                    if (typeof options.error === 'function')
                        options.error(error)
                })
            },
            pause: function(options) {
                let self = this
                if (!options) options = {}

                api.services.performances.pause.callService({}, function(response) {
                    if (response.success) {
                        self.trigger('pause', response.time)
                        self.resumeTime = response.time
                        if (typeof options.success === 'function')
                            options.success(response)
                    } else if (typeof options.error === 'function')
                        options.error('Unable to pause performance')
                }, function(error) {
                    if (typeof options.error === 'function')
                        options.error(error)
                })
            },
            b_pause: function(options) {
                // Paused by backend script
                this.trigger('pause', options.time)
                this.resumeTime = options.time
            },
            stop: function(options) {
                let self = this
                if (!options) options = {}

                api.services.performances.stop.callService({}, function(response) {
                    if (response.success) {
                        self.trigger('stop', response.time)
                        if (typeof options.success === 'function')
                            options.success(response)
                    } else if (typeof options.error === 'function')
                        options.error('Unable to stop performance')
                }, function(error) {
                    if (typeof options.error === 'function')
                        options.error(error)
                })
            },
            updateDuration: function() {
                let self = this
                this.duration = 0

                if (this.nodes.length > 0) {
                    this.nodes.forEach(function(node) {
                        if (node.get('name') === 'pause')
                            node.set('duration', 0.1)

                        let startTime = node.get('start_time'),
                            nodeDuration = node.get('duration')
                        if (startTime !== null && nodeDuration !== null && startTime + nodeDuration > self.duration)
                            self.duration = startTime + nodeDuration
                    })
                }
            },
            getDuration: function() {
                if (!this.duration) this.updateDuration()
                return this.duration
            },
            getDescription: function() {
                let desc = ''
                this.nodes.forEach(function(node) {
                    if (node.get('name') === 'speech')
                        desc += node.get('text') + ' '
                })
                return desc.trim()
            },
            handleEvents: function(msg) {
                if (msg.event = 'paused') {
                    this.trigger('pause', msg.time)
                }
            },
            mergeNodes: function(performances) {
                let nodes = []
                performances.forEach(function(p) {
                    nodes = _.union(nodes, p['nodes'])
                })
                return nodes
            }
        }, {
            // static methods
            unload: function(options) {
                options = options || {}
                api.services.performances.unload.callService({}, function(response) {
                    if (response.success) {
                        if (typeof options.success === 'function')
                            options.success(response)
                    } else if (typeof options.error === 'function')
                        options.error('Error unloading performance')
                }, function(error) {
                    if (typeof options.error === 'function')
                        options.error(error)
                })
            },
        })
    })
