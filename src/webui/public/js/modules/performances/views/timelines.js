define(['application', 'tpl!./templates/timelines.tpl', 'd3', './timeline', './node',
    'lib/extensions/animate_auto'], function (App, template, d3) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Timelines = Marionette.CompositeView.extend({
            template: template,
            childView: App.Performances.Views.Timeline,
            childViewContainer: '.app-timelines',
            config: {
                pxPerSec: 70
            },
            ui: {
                timelines: '.app-timelines',
                nodes: '.app-nodes .app-node',
                nodesContainer: '.app-nodes',
                nodeSettings: '.app-node-settings',
                scrollContainer: '.app-scroll-container',
                performanceName: '.app-performance-name',
                saveButton: '.app-save-button',
                runButton: '.app-run-button',
                stopButton: '.app-stop-button',
                pauseButton: '.app-pause-button',
                resumeButton: '.app-resume-button',
                loopButton: '.app-loop-button',
                clearButton: '.app-clear-button',
                runIndicator: '.app-run-indicator',
                deleteButton: '.app-delete-button',
                timeAxis: '.app-time-axis'
            },
            events: {
                'click @ui.nodes': 'nodeClicked',
                'click @ui.saveButton': 'savePerformances',
                'keyup @ui.performanceName': 'setPerformanceName',
                'click @ui.runButton': 'run',
                'click @ui.stopButton': 'stop',
                'click @ui.pauseButton': 'pause',
                'click @ui.resumeButton': 'resume',
                'click @ui.clearButton': 'clearPerformance',
                'click @ui.deleteButton': 'deletePerformance',
                'click @ui.timeAxis': 'runAt',
                'click @ui.loopButton': 'loop'
            },
            onShow: function () {
                var self = this;

                this.model.stop();
                this.resetButtons();
                this.ui.runIndicator.hide();
                this.model.get('nodes').each(function (node) {
                    self.createNodeEl(node);
                });

                this.arrangeNodes();

                this.model.get('nodes').bind('add', this.addNode, this);
                this.model.get('nodes').bind('remove', this.arrangeNodes, this);

                // add resize event
                $(window).resize(function () {
                    self.updateTimelineWidth();
                });

                this.model.on('run', function (time, timestamp) {
                    console.log(time);
                    console.log(timestamp);
                });
            },
            onDestroy: function () {
                this.model.stop();
                this.model.get('nodes').unbind('add', this.addNode, this);
                this.model.get('nodes').unbind('remove', this.arrangeNodes, this);

                // remove resize event
                $(window).off('resize');

                this.model.get('nodes').each(function (node) {
                    $(node.get('el')).remove();
                });
            },
            nodeClicked: function (e) {
                var node = new App.Performances.Entities.Node({
                    name: $(e.target).data('name'),
                    start_time: 0,
                    duration: 1
                });

                this.model.get('nodes').add(node);
                this.showNodeSettings(node);
            },
            createNodeEl: function (node) {
                var self = this,
                    el = $('.app-node[data-name="' + node.get('name') + '"]', this.ui.nodesContainer).clone().get(0);

                // show node settings on click
                $(el).on('click', function () {
                    self.showNodeSettings(node);
                }).hover(function () {
                    $(this).animateAuto('width', 500);
                }, function () {
                    self.updateNode(node);
                });

                node.set('el', el);
                node.on('change', this.updateNode, this);
                // Listen for Pause nodes
                this.listenTo(node, 'pause', function () {
                    this.pause();
                });

                this.listenTo(node, 'resume', function () {
                    this.run();
                });
                this.updateNode(node);
            },
            /**
             * Updates node element
             *
             * @param node App.Performances.Entities.Node
             */
            updateNode: function (node) {
                var width = node.get('duration') * this.config.pxPerSec;
                $(node.get('el')).stop().animate({
                    left: node.get('start_time') * this.config.pxPerSec,
                    width: width,
                    minWidth: width
                });

                if (node.changed['duration'] || node.changed['start_time']) {
                    this.arrangeNodes();
                }

                if (node.get('text'))
                    $(node.get('el')).html(node.get('text'));
                else if (node.get('emotion'))
                    $(node.get('el')).html(node.get('emotion'));
                else if (node.get('gesture'))
                    $(node.get('el')).html(node.get('gesture'));
                else if (node.get('expression'))
                    $(node.get('el')).html(node.get('expression'));
            },
            addNode: function (node) {
                this.createNodeEl(node);
                this.arrangeNodes();
            },
            showNodeSettings: function (node) {
                var self = this,
                    oldView = null;

                if (typeof this.nodeView != 'undefined')
                    oldView = this.nodeView;

                this.nodeView = new Views.Node({model: node});
                this.nodeView.render();

                this.ui.nodeSettings.slideUp(function () {
                    self.ui.nodeSettings.html(self.nodeView.el).hide().slideDown();
                    if (oldView)
                        oldView.destroy();
                });

                $(node.get('el')).addClass('active');
                this.nodeView.on('destroy', function () {
                    $(node.get('el')).removeClass('active');
                });
            },
            arrangeNodes: function () {
                var self = this,
                    available = false;

                this.model.get('nodes').each(function (node) {
                    var begin = node.get('start_time'),
                        end = begin + node.get('duration');

                    self.children.every(function (view) {
                        available = true;

                        _.every($('.app-node', view.ui.nodes), function (el) {
                            var model = self.model.get('nodes').findWhere({'el': el});

                            if (model && model != node) {
                                var compareBegin = model.get('start_time'),
                                    compareEnd = compareBegin + model.get('duration');

                                // check if intersects
                                if ((compareBegin <= begin && compareEnd > begin) ||
                                    (compareBegin < end && compareEnd >= end) ||
                                    (compareBegin <= begin && compareEnd >= end)
                                ) {
                                    available = false;
                                    return false;
                                }
                            }

                            return true;
                        });

                        if (available) {
                            view.ui.nodes.append(node.get('el'));
                            return false;
                        }

                        return true;
                    });

                    if (!available) {
                        self.addTimeline();
                        self.children.last().ui.nodes.append(node.get('el'));
                    }
                });

                // remove empty timelines
                while (this.children.last() && $('.app-node', this.children.last().ui.nodes).length == 0)
                    this.collection.pop();

                this.updateTimelineWidth();
            },
            addTimeline: function () {
                this.collection.add(new Backbone.Model({}));
            },
            getTimelineWidth: function () {
                return this.model.getDuration() * this.config.pxPerSec;
            },
            updateTimelineWidth: function () {
                if (this.children.length > 0) {
                    var width = this.getTimelineWidth(),
                        containerWidth = this.ui.timelines.width(),
                        scaleWidth = Math.max(width, containerWidth),
                        scale = d3.scale.linear().domain([0, scaleWidth / this.config.pxPerSec]).range([0, scaleWidth]);

                    // update axis
                    this.ui.timeAxis.html('').width(scaleWidth);
                    d3.select(this.ui.timeAxis.get(0)).call(d3.svg.axis().scale(scale).orient("bottom"));

                    if (width < containerWidth)
                        width = '100%';

                    this.children.each(function (timeline) {
                        timeline.ui.nodes.css('width', width);
                    });
                }
            },
            setPerformanceName: function () {
                this.model.set('name', this.ui.performanceName.val());
            },
            savePerformances: function () {
                if (typeof this.options.performances != 'undefined' && !this.options.performances.contains(this.model))
                    this.options.performances.add(this.model);

                Views.trigger('performances:save');
            },
            startIndicator: function (startTime, endTime, callback) {
                var self = this;
                this.ui.runButton.hide();
                this.ui.resumeButton.hide();
                this.ui.stopButton.fadeIn();
                this.ui.pauseButton.fadeIn();

                $(this.ui.runIndicator).stop().css('left', startTime * this.config.pxPerSec).show()
                    .animate({left: endTime * this.config.pxPerSec}, (endTime - startTime) * 1000, 'linear',
                        function () {
                            if (self.isDestroyed) return;
                            if (typeof callback == 'function')
                                callback();
                        });
            },
            resetButtons: function () {
                this.ui.runButton.fadeIn();
                this.ui.stopButton.hide();
                this.ui.pauseButton.hide();
                this.ui.resumeButton.hide();
            },
            pauseIndicator: function (time) {
                $(this.ui.runIndicator).stop().css('left', time * this.config.pxPerSec);
                this.ui.pauseButton.hide();
                this.ui.resumeButton.fadeIn();
            },
            stopIndicator: function () {
                this.ui.runIndicator.hide().stop();
                this.resetButtons();
                if (this.enableLoop)
                    this.run();
            },
            run: function (startTime) {
                var self = this,
                    duration = this.model.getDuration();

                if (!$.isNumeric(startTime))
                    startTime = 0;

                this.model.run(startTime, {
                    success: function (response) {
                        var startTime = response.time - (response.timestamp - (Date.now() / 1000));
                        self.startIndicator(startTime, response.time, function () {
                            if (response.time < duration)
                                self.pauseIndicator(response.time);
                            else
                                self.stopIndicator();
                        });
                    },
                    error: function (error) {
                        console.log(error);
                    }
                });
            },
            stop: function () {
                var self = this;
                this.loop(false);
                this.model.stop({
                    success: function () {
                        self.stopIndicator();
                    },
                    error: function (error) {
                        console.log(error);
                    }
                });
            },
            pause: function () {
                var self = this;
                this.model.pause({
                    success: function (response) {
                        if (response.success)
                            self.pauseIndicator(response.time);
                        else
                            console.log('Performance not running');
                    },
                    error: function (error) {
                        console.log(error);
                    }
                });
            },
            resume: function () {
                var self = this,
                    duration = this.model.getDuration();

                this.model.resume({
                    success: function (response) {
                        if (response.success) {
                            var startTime = response.time - (response.timestamp - (Date.now() / 1000));
                            self.startIndicator(startTime, response.time, function () {
                                if (response.time < duration)
                                    self.pauseIndicator(response.time);
                                else
                                    self.stopIndicator();
                            });
                        } else
                            console.log('Performance not running or not paused');
                    },
                    error: function (error) {
                        console.log(error);
                    }
                });
            },
            runAt: function (e) {
                this.run(e.offsetX / this.config.pxPerSec);
            },
            loop: function (enable) {
                if (typeof enable == 'boolean' && !enable || this.enableLoop) {
                    this.enableLoop = false;
                    this.ui.loopButton.removeClass('active').blur();
                } else {
                    this.enableLoop = true;
                    this.ui.loopButton.addClass('active');
                }
            },
            /**
             * Removes all nodes from the performance
             */
            clearPerformance: function () {
                var self = this,
                    nodes = [];

                this.children.each(function (view) {
                    // store node el's
                    nodes = _.union(nodes, $('.app-node', view.el).toArray());

                    // remove timeline model
                    self.collection.remove(view.model);
                });

                // delete nodes
                _.each(nodes, function (el) {
                    var node = self.model.get('nodes').findWhere({'el': el});
                    if (node) node.destroy();
                });
            },
            deletePerformance: function () {
                var self = this;

                Views.trigger('performance:delete', this.model);

                this.$el.fadeOut(null, function () {
                    self.model.destroy();
                    self.savePerformances();
                    self.destroy();
                })
            }
        });
    });
});
