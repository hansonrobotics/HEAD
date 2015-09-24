define(['application', 'lib/api', './node'], function (App, api) {
    App.module('Performances.Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Performance = Backbone.Model.extend({
            initialize: function (options) {
                var self = this,
                    nodes = new App.Performances.Entities.NodeCollection();
                if (options && options.nodes) {
                    _.each(options.nodes, function (attrs) {
                        nodes.add(new Entities.Node(attrs));
                    });
                }

                nodes.on('change', function () {
                    self.trigger('change');
                });

                this.set('nodes', nodes);
            },
            run: function (startTime) {
                this.stop();
                var self = this;

                // set start time
                if (typeof startTime == 'undefined' && $.isNumeric(this.getResumeTime())) {
                    startTime = this.getResumeTime();
                } else if (!$.isNumeric(startTime)) {
                    startTime = 0;
                }

                this.lastRunTime = Date.now() - (startTime * 1000);
                this.resumeStartTime = null;

                this.get('nodes').each(function (node) {
                    if (node.get('start_time') >= startTime)
                        self.runQueue.push(setTimeout(function () {
                            node.call();
                        }, (node.get('start_time') - startTime) * 1000));
                });
            },
            pause: function () {
                if ($.isNumeric(this.lastRunTime)) {
                    this.resumeStartTime = Date.now() - this.lastRunTime;

                    _.each(this.runQueue, function (timeout) {
                        clearTimeout(timeout);
                    });

                    this.runQueue = [];
                }
            },
            stop: function () {
                _.each(this.runQueue, function (timeout) {
                    clearTimeout(timeout);
                });

                this.runQueue = [];
                this.lastRunTime = null;
            },
            /**
             * Returns resume time in seconds if paused or null otherwise
             *
             * @returns {*}
             */
            getResumeTime: function () {
                if ($.isNumeric(this.resumeStartTime)) {
                    return this.resumeStartTime / 1000;
                } else {
                    return null;
                }
            },
            getDuration: function () {
                var duration = 0;

                if (this.get('nodes')) {
                    this.get('nodes').each(function (node) {
                        duration = Math.max(duration, node.get('start_time') + node.get('duration'));
                    });
                }

                return duration;
            }
        });
        Entities.PerformanceCollection = Backbone.Collection.extend({
            model: Entities.Performance,
            fetch: function () {
                var self = this;
                $.ajax('/performances/get/' + api.config.robot, {
                    success: function (data) {
                        var json = JSON.parse(data);

                        _.each(json, function (attrs) {
                            self.add(new Entities.Performance(attrs));
                        });
                    }
                });
            },
            save: function (successCallback, errorCallback) {
                var data = [];

                this.each(function (model) {
                    var performance = model.toJSON();
                    performance['nodes'] = model.get('nodes').toJSON();
                    data.push(performance);
                });

                $.ajax('/performances/update/' + api.config.robot, {
                    data: JSON.stringify(data),
                    type: 'POST',
                    dataType: "json",
                    success: function () {
                        if (typeof successCallback == 'function')
                            successCallback();
                    },
                    error: function () {
                        if (typeof errorCallback == 'function')
                            errorCallback();
                    }
                });
            }
        });
    });
});
