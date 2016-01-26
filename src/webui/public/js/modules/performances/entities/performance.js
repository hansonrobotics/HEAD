define(['application', 'lib/api', './node'], function (App, api) {
    App.module('Performances.Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Performance = Backbone.Model.extend({
            urlRoot: function () {
                return '/performances/' + api.config.robot + '/' + this.get('slug');
            },
            initialize: function (options) {
                var self = this,
                    nodes = new App.Performances.Entities.NodeCollection();

                this.set('nodes', nodes);

                if (options && options.nodes) {
                    _.each(options.nodes, function (attributes) {
                        nodes.add(new Entities.Node(attributes));
                    });
                }

                // trigger performance change on node change
                nodes.on('change', function () {
                    self.trigger('change');
                });

                // update slug
                this.on('change:name', function () {
                    self.handleFilename();
                });

            },
            /**
             * Run after data fetch from server
             *
             * @param response
             * @returns {*}
             */
            parse: function (response) {
                // remove previous slug
                delete response['previous_slug'];
                this.unset('previous_slug');

                // keep instance of NodeCollection at nodes attribute
                if (this.get('nodes') instanceof App.Performances.Entities.NodeCollection) {
                    // do not update node collection
                    delete response['nodes'];
                }

                return response;
            },
            handleFilename: function () {
                if (this.changed['name'])
                    this.set('slug', this.get('name').toLowerCase()
                        .replace(/ /g, '_')
                        .replace(/[^\w-]+/g, '')
                    );

                if (!this.get('previous_slug') && this.changed['slug'])
                    this.set('previous_slug', this.previous('slug'))
            },
            run: function (startTime, options) {
                var self = this;
                if (!options) options = {};

                api.services.performances.run.callService({
                    nodes: JSON.stringify(this.get('nodes').toJSON()),
                    startTime: startTime
                }, function (response) {
                    if (response.success) {
                        self.paused = false;
                        self.startTime = response.time - (response.timestamp - (Date.now() / 1000));
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
                        self.trigger('resume', response.time, response.timestamp);
                        self.paused = false;
                        self.resumeTime = null;
                        self.startTime = response.time - (response.timestamp - (Date.now() / 1000));

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
                        self.trigger('pause', response.time, response.timestamp);
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
            stop: function (options) {
                var self = this;
                if (!options) options = {};

                api.services.performances.stop.callService({}, function (response) {
                    if (response.success) {
                        self.trigger('stop', response.time, response.timestamp);

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
                var nodes = this.get('nodes'),
                    duration = 0;
                if (nodes && nodes.length > 0) {
                    nodes.forEach(function(node){
                        var startTime = node.get('start_time'),
                            nodeDuration = node.get('duration');
                        if (startTime + nodeDuration > duration){
                            duration = startTime + nodeDuration
                        }
                    });
                return duration;
            },
            getRunTime: function () {
                if (this.start_time)
                    return Date.now() - this.start_time;
            },
            getResumeTime: function () {
                if ($.isNumeric(this.resumeTime))
                    return this.resumeTime;
                else
                    return null;
            }
        });
        Entities.PerformanceCollection = Backbone.Collection.extend({
            model: Entities.Performance,
            url: '/performances/' + api.config.robot
        });
    });
});
