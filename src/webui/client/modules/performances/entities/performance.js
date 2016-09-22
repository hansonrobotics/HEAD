define(['application', 'backbone', 'lib/api', './node_collection', 'underscore', './node'],
    function (App, Backbone, api, NodeCollection, _) {
        return Backbone.Model.extend({
            urlRoot: function () {
                console.log(api.config.robot);
                return '/performances/' + api.config.robot;
            },
            initialize: function (options) {
                var self = this;

                this.nodes = new NodeCollection();
                this.set('nodes', this.nodes);

                if (options && options.nodes)
                    _.each(options.nodes, function (attributes) {
                        delete attributes['id'];
                        self.nodes.add(new self.nodes.model(attributes));
                    });

                // trigger performance change on node change
                this.nodes.on('change', function () {
                    self.trigger('change:nodes');
                });

                // making sure nodes attribute holds NodeCollection instance
                this.on('change:nodes', function () {
                    if (this.get('nodes') != this.nodes) {
                        // updating collection
                        this.nodes.set(this.get('nodes'));
                        this.set({nodes: this.nodes}, {silent: true});
                    }
                });

                // update id
                this.on('change:name change:path', this.updateId);
                this.updateId();
            },
            fetchCurrent: function (options) {
                var self = this;
                options = options || {};
                api.services.performances.current.callService({}, function (response) {
                    self.nodes.reset(JSON.parse(response.nodes));

                    if (typeof options.success == 'function')
                        options.success(response);
                }, function (error) {
                    if (typeof options.error == 'function')
                        options.error(error);
                });
            },
            load: function (options) {
                this.loadSequence([this.id], options);
            },
            loadNodes: function (options) {
                options = options || {};
                api.services.performances.load_nodes.callService({
                    nodes: JSON.stringify(this.nodes.toJSON())
                }, function (response) {
                    if (response.success) {
                        if (typeof options.success == 'function')
                            options.success(response);
                    } else if (typeof options.error == 'function')
                        options.error('Another performance is running');
                }, function (error) {
                    if (typeof options.error == 'function')
                        options.error(error);
                });
            },
            loadSequence: function (ids, options) {
                var self = this;
                options = options || {};
                api.services.performances.load_sequence.callService({
                    ids: ids
                }, function (response) {
                    if (response.success) {
                        self.nodes.reset(JSON.parse(response.nodes));

                        if (typeof options.success == 'function')
                            options.success(response);
                    } else if (typeof options.error == 'function')
                        options.error('Another performance is running');
                }, function (error) {
                    if (typeof options.error == 'function')
                        options.error(error);
                });
            },
            parse: function (response) {
                delete response['previous_id'];
                this.unset('previous_id');
                this.unset('ignore_nodes');
                return response;
            },
            updateId: function () {
                if (this.get('name')) {
                    var name = this.get('name').toLowerCase().replace(/ /g, '_').replace(/[^\w-]+/g, ''),
                        path = this.get('path') ? this.get('path').split('/') : [];

                    path.push(name);

                    this.set('id', path.join('/'));

                    if (!this.get('previous_id') && this.previous('id'))
                        this.set('previous_id', this.previous('id'));
                }
            },
            run: function (startTime, options) {
                var self = this;
                if (!options) options = {};

                api.services.performances.run.callService({
                    startTime: startTime
                }, function (response) {
                    if (response.success) {
                        self.paused = false;
                        self.startTime = response.time;
                        self.trigger('run', startTime);

                        if (typeof options.success == 'function')
                            options.success(response);
                    } else if (typeof options.error == 'function')
                        options.error('Another performance is running');
                }, function (error) {
                    if (typeof options.error == 'function')
                        options.error(error);
                });
            },
            resume: function (options) {
                var self = this;
                if (!options) options = {};

                api.services.performances.resume.callService({}, function (response) {
                    if (response.success) {
                        self.trigger('resume', self.resumeTime);
                        self.paused = false;
                        response.time = self.resumeTime;
                        self.resumeTime = null;
                        if (typeof options.success == 'function')
                            options.success(response);
                    } else if (typeof options.error == 'function')
                        options.error('Unable to resume performance');
                }, function (error) {
                    if (typeof options.error == 'function')
                        options.error(error);
                });
            },
            pause: function (options) {
                var self = this;
                if (!options) options = {};

                api.services.performances.pause.callService({}, function (response) {
                    if (response.success) {
                        self.trigger('pause', response.time);
                        self.resumeTime = response.time;
                        if (typeof options.success == 'function')
                            options.success(response);
                    } else if (typeof options.error == 'function')
                        options.error('Unable to pause performance');
                }, function (error) {
                    if (typeof options.error == 'function')
                        options.error(error);
                });
            },
            b_pause: function (options) {
                // Paused by backend script
                this.trigger('pause', options.time);
                this.resumeTime = options.time;
            },
            stop: function (options) {
                var self = this;
                if (!options) options = {};

                api.services.performances.stop.callService({}, function (response) {
                    if (response.success) {
                        self.trigger('stop', response.time);
                        if (typeof options.success == 'function')
                            options.success(response);
                    } else if (typeof options.error == 'function')
                        options.error('Unable to stop performance');
                }, function (error) {
                    if (typeof options.error == 'function')
                        options.error(error);
                });
            },
            getDuration: function () {
                var duration = 0;

                if (this.nodes.length > 0) {
                    this.nodes.forEach(function (node) {
                        var startTime = node.get('start_time'),
                            nodeDuration = node.get('duration');

                        if (startTime != null && nodeDuration != null && startTime + nodeDuration > duration)
                            duration = startTime + nodeDuration;
                    });
                }

                return duration;
            },
            handleEvents: function (msg) {
                if (msg.event = 'paused') {
                    this.trigger('pause', msg.time);
                }
            }
        });
    });
