define(['application', 'marionette', 'tpl!./templates/timelines.tpl', 'd3', 'bootbox', './timeline', './node',
        '../entities/node', 'lib/extensions/animate_auto', 'jquery-ui', 'scrollbar'],
    function (App, Marionette, template, d3, bootbox, TimelineView, NodeView, Node) {
        return Marionette.CompositeView.extend({
            template: template,
            childView: TimelineView,
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
                frameCount: '.app-frame-count',
                timeIndicator: '.app-current-time div',
                deleteButton: '.app-delete-button',
                timeAxis: '.app-time-axis',
                hideButton: '.app-hide-settings-button'
            },
            events: {
                'click @ui.nodes': 'nodeClicked',
                'click @ui.saveButton': 'savePerformances',
                'keyup @ui.performanceName': 'setPerformanceName',
                'click @ui.runButton': 'runAtIndicator',
                'click @ui.stopButton': 'stop',
                'click @ui.pauseButton': 'pause',
                'click @ui.resumeButton': 'resume',
                'click @ui.timeAxis': 'moveIndicator',
                'click @ui.clearButton': 'clearPerformance',
                'click @ui.deleteButton': 'deletePerformance',
                'click @ui.loopButton': 'loop',
                'click @ui.hideButton': 'hideNodeSettings'
            },
            onShow: function () {
                var self = this;
                this.ui.scrollContainer.perfectScrollbar();
                // Performance event handler
                if (typeof this.options.performances != 'undefined') {
                    this.options.performances.eventHandler = function (msg) {
                        self.handleEvents(msg);
                    }
                }
                if (this.options.readonly){
                    this.ui.nodesContainer.hide();
                }
                this.model.get('nodes').on('add remove', function () {
                    if (self.model.get('nodes').isEmpty())
                        self.ui.clearButton.stop().fadeOut();
                    else
                        self.ui.clearButton.stop().fadeIn();
                });

                this.ui.hideButton.hide();
                // hide delete and clear buttons for new models
                if (!this.model.get('id'))
                    this.ui.deleteButton.hide();
                    this.ui.clearButton.hide();

                this.stopIndicator();
                this.model.get('nodes').each(function (node) {
                    self.createNodeEl(node);
                });
                this.arrangeNodes();
                this.model.get('nodes').bind('add', this.addNode, this);
                this.model.get('nodes').bind('remove', this.arrangeNodes, this);

                // add resize event
                var updateWidth = function () {
                    if (self.isDestroyed)
                        $(window).off('resize', updateWidth);
                    else
                        self.updateTimelineWidth();
                };
                $(".app-timelines", this.el).droppable({
                    accept: ".app-node"
                });
                $(window).on('resize', updateWidth);
            },
            onDestroy: function () {
                this.stopIndicator();

                this.model.stop();
                this.model.get('nodes').unbind('add', this.addNode, this);
                this.model.get('nodes').unbind('remove', this.arrangeNodes, this);
                this.model.get('nodes').each(function (node) {
                    $(node.get('el')).remove();
                });
                if (typeof this.options.performances != 'undefined') {
                    this.options.performances.eventHandler = false;
                }
            },
            nodeClicked: function (e) {
                var node = new Node({
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
                }).draggable({
                    snap: '.app-.app-timeline-nodes',
                    revert: 'invalid',
                    delay: 300,
                    disabled: self.options.readonly,
                    stop: function(e, ui){
                        node.set('start_time', parseInt($(this).css('left'))/self.config.pxPerSec);
                        $(this).css({top:0});
                        self.arrangeNodes();
                    }
                }).css({position:'absolute'});

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
             * @param node Node
             */
            updateNode: function (node) {
                var width = node.get('duration') * this.config.pxPerSec;
                $(node.get('el')).stop().animate({
                    left: node.get('start_time') * this.config.pxPerSec,
                    width: width,
                    minWidth: width
                });

                if (node.changed['duration'] || node.changed['start_time'])
                    this.arrangeNodes();

                if (node.get('text'))
                    $(node.get('el')).html(node.get('text'));
                else if (node.get('emotion'))
                    $(node.get('el')).html(node.get('emotion'));
                else if (node.get('gesture'))
                    $(node.get('el')).html(node.get('gesture'));
                else if (node.get('expression'))
                    $(node.get('el')).html(node.get('expression'));
                else if (node.get('animation'))
                    $(node.get('el')).html(node.get('animation'));
            },
            addNode: function (node) {
                this.createNodeEl(node);
                this.arrangeNodes();
            },
            showNodeSettings: function (node) {
                if (this.options.readonly){
                    return
                }
                var self = this,
                    showSettings = !this.nodeView || this.nodeView.model != node;

                this.ui.timelines.find('.app-node').removeClass('active');
                if (showSettings)
                    $(node.get('el')).addClass('active');

                this.ui.nodeSettings.slideUp(function () {
                    if (self.nodeView)
                        self.hideNodeSettings();

                    // show settings if no settings are shown or if shown settings are for a different node
                    if (showSettings) {
                        self.nodeView = new NodeView({model: node});
                        self.nodeView.render();
                        self.ui.nodeSettings.html(self.nodeView.el).hide().slideDown(function () {
                            self.ui.hideButton.fadeIn();
                        });
                    }
                });
            },
            hideNodeSettings: function () {
                var self = this,
                    destroy = function () {
                        self.ui.timelines.find('.app-node').removeClass('active');
                        self.ui.hideButton.fadeOut();
                        if (self.nodeView) {
                            self.nodeView.destroy();
                            self.nodeView = null;
                        }
                    };
                if (this.ui.nodeSettings.is(':visible')) {
                    self.ui.nodeSettings.slideUp(function () {
                        destroy();
                    });
                } else {
                    destroy();
                }
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

                    this.ui.scrollContainer.perfectScrollbar('update');
                }
            },
            setPerformanceName: function () {
                this.model.set('name', this.ui.performanceName.val());
            },
            savePerformances: function () {
                var self = this;

                if (typeof this.options.performances != 'undefined' && !this.options.performances.contains(this.model))
                    this.options.performances.add(this.model);

                this.model.save({}, {
                    success: function () {
                        self.ui.deleteButton.fadeIn();
                        self.ui.clearButton.fadeIn();
                    }
                });
            },
            startIndicator: function (startTime, endTime, callback) {
                var self = this;
                this.running = true;
                this.paused = false;
                this.ui.runButton.hide();
                this.ui.resumeButton.hide();
                this.ui.stopButton.fadeIn();
                this.ui.pauseButton.fadeIn();

                if (this.ui.runIndicator.draggable('instance'))
                    this.ui.runIndicator.draggable('destroy');

                this.ui.runIndicator.stop().css('left', startTime * this.config.pxPerSec).show()
                    .animate({left: endTime * this.config.pxPerSec}, {
                        duration: (endTime - startTime) * 1000,
                        easing: 'linear',
                        step: function () {
                            self.updateIndicatorTime();
                        },
                        complete: function () {
                            if (self.isDestroyed) return;
                            if (typeof callback == 'function')
                                callback();
                        }
                    });
            },
            updateIndicatorTime: function (time) {
                if (!$.isNumeric(time))
                    time = parseInt(this.ui.runIndicator.css('left')) / this.config.pxPerSec;

                var step = 1. / App.getOption('fps'),
                    frameCount = parseInt(time / step);

                this.ui.frameCount.html(frameCount);
                this.ui.timeIndicator.html((frameCount * step).toFixed(2));
            },
            resetButtons: function () {
                this.ui.runButton.fadeIn();
                this.ui.stopButton.hide();
                this.ui.pauseButton.hide();
                this.ui.resumeButton.hide();
            },
            pauseIndicator: function (time) {
                this.paused = true;
                this.ui.runIndicator.stop().css('left', time * this.config.pxPerSec);
                this.pausePosition = parseInt(this.ui.runIndicator.css('left'));
                this.updateIndicatorTime();
                this.enableIndicatorDragging();
                this.ui.pauseButton.hide();
                this.ui.resumeButton.fadeIn();
            },
            enableIndicatorDragging: function () {
                var self = this;
                this.ui.runIndicator.draggable({
                    axis: "x",
                    drag: function () {
                        self.updateIndicatorTime();
                    },
                    stop: function (event, ui) {
                        var endPixels = self.model.getDuration() * self.config.pxPerSec;
                        if (ui.position.left < 0) {
                            self.ui.runIndicator.animate({left: 0});
                            self.updateIndicatorTime(0);
                        } else if (ui.position.left > endPixels) {
                            self.ui.runIndicator.animate({left: endPixels});
                            self.updateIndicatorTime(endPixels / self.config.pxPerSec);
                        }
                    }
                });
            },
            stopIndicator: function () {
                this.running = false;
                this.paused = false;
                this.ui.runIndicator.stop().css('left', 0);
                this.enableIndicatorDragging();
                this.updateIndicatorTime(0);
                this.resetButtons();

                if (this.enableLoop)
                    this.run();
            },
            moveIndicator: function (e) {
                if (!this.running || this.paused) {
                    this.ui.runIndicator.css('left', Math.min(e.offsetX, this.model.getDuration() * this.config.pxPerSec));
                    this.updateIndicatorTime();
                }
            },
            run: function (startTime) {
                if (!$.isNumeric(startTime))
                    startTime = 0;

                this.model.run(startTime, {
                    error: function (error) {
                        console.log(error);
                    }
                });
            },
            stop: function () {
                this.loop(false);
                this.model.stop({
                    error: function (error) {
                        console.log(error);
                    }
                });
            },
            pause: function () {
                this.model.pause({
                    error: function (error) {
                        console.log(error);
                    }
                });
            },
            resume: function () {
                if (this.paused && parseInt(this.ui.runIndicator.css('left')) == this.pausePosition)
                    this.model.resume({
                        error: function (error) {
                            console.log(error);
                        }
                    });
                else
                    this.runAtIndicator();
            },
            runAtIndicator: function () {
                this.run(1. * parseInt(this.ui.runIndicator.css('left')) / this.config.pxPerSec);
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
                    view.model.destroy();
                });

                // delete nodes
                _.each(nodes, function (el) {
                    var node = self.model.get('nodes').findWhere({'el': el});
                    if (node) node.destroy();
                });
            },
            deletePerformance: function () {
                var self = this;

                bootbox.confirm("Are you sure?", function (result) {
                    if (result)
                        self.$el.fadeOut(null, function () {
                            self.model.destroy();
                            self.destroy();
                        });
                });
            },
            handleEvents: function (e) {
                var duration = this.model.getDuration();

                if (e.event == 'paused') {
                    this.pauseIndicator(e.time);
                    this.model.b_pause(e);
                } else if (e.event == 'idle') {
                    this.stopIndicator();
                } else if (e.event == 'running') {
                    this.startIndicator(e.time, duration);
                } else if (e.event == 'resume') {
                    this.startIndicator(e.time, duration);
                }
            }
        });
    });
