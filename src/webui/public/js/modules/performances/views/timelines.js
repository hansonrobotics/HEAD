define(['application', 'marionette', 'tpl!./templates/timelines.tpl', 'd3', 'bootbox', './node',
        '../entities/node', 'underscore', 'jquery', '../entities/performance', 'lib/extensions/animate_auto',
        'jquery-ui', 'scrollbar'],
    function (App, Marionette, template, d3, bootbox, NodeView, Node, _, $, Performance) {
        return Marionette.LayoutView.extend({
            template: template,
            cssClass: 'app-timeline-editor-container',
            config: {
                pxPerSec: 70
            },
            ui: {
                timelineContainer: '.app-timelines',
                timelineNodes: '.app-timeline-nodes .app-node',
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
            modelEvents: {
                'change': 'modelChanged'
            },
            initialize: function (options) {
                var self = this,
                    addReloadHandler = {success: function () {
                        var handler = function () {
                            if (self.isDestroyed)
                                self.model.nodes.off('change add remove', handler);
                            else
                                self.model.loadNodes();
                        };

                        self.model.nodes.on('change add remove', handler);
                    }};

                this.mergeOptions(options, ['performances']);

                if (options.sequence instanceof Array) {
                    this.model = new Performance();
                    this.model.loadSequence(options.sequence, addReloadHandler);
                } else if (this.model) {
                    if (this.model.id && this.model.nodes.isEmpty()) {
                        this.model.load(addReloadHandler);
                    } else
                        this.model.loadNodes(addReloadHandler);
                } else {
                    this.model = new Performance();
                    this.model.fetchCurrent(addReloadHandler);
                }
            },
            childViewOptions: function () {
                return {performance: this.model, config: this.config};
            },
            onAttach: function () {
                var self = this;
                this.modelChanged();
                this.initDraggable(this.ui.nodes);
                this.ui.scrollContainer.perfectScrollbar();
                // Performance event handler
                if (typeof this.options.performances != 'undefined')
                    this.options.performances.eventHandler = function (msg) {
                        self.handleEvents(msg);
                    };

                if (this.options.readonly)
                    this.ui.nodesContainer.hide();

                var nodeListener = function () {
                    if (self.isDestroyed)
                        this.off('add remove reset', nodeListener);
                    else if (self.model.nodes.isEmpty())
                        self.ui.clearButton.fadeOut();
                    else if (!self.options.readonly)
                        self.ui.clearButton.fadeIn();
                };
                this.model.nodes.on('add remove reset', nodeListener);

                // hide delete and clear buttons for new models
                if (!this.model.get('id')) {
                    this.ui.deleteButton.hide();
                    this.ui.clearButton.hide();
                }

                this.stopIndicator();
                this.removeNodeElements();
                this.arrangeNodes();
                this.model.nodes.bind('add', this.addNode, this);
                this.model.nodes.bind('remove reset', this.arrangeNodes, this);

                // add resize event
                var updateWidth = function () {
                    if (self.isDestroyed)
                        $(window).off('resize', updateWidth);
                    else
                        self.updateTimelineWidth();
                };
                $(window).on('resize', updateWidth);

                // init node droppable
                this.ui.scrollContainer.droppable({
                    accept: '.app-node[data-name]',
                    tolerance: 'touch',
                    drop: function (e, ui) {
                        var nodeEl = $(ui.helper),
                            node = self.model.nodes.get({cid: nodeEl.data('cid')}),
                            startTime = Math.round(
                                    ($(this).scrollLeft() + ui.offset.left - $(this).offset().left) / self.config.pxPerSec * 100) / 100;

                        if (node)
                            node.set('start_time', startTime);
                        else
                            self.model.nodes.add({
                                name: nodeEl.data('name'),
                                start_time: startTime,
                                duration: 1
                            });
                    }
                });
            },
            onDestroy: function () {
                this.stopIndicator();

                this.model.stop();
                this.model.nodes.unbind('add', this.addNode, this);
                this.model.nodes.unbind('remove reset', this.arrangeNodes, this);
                this.model.nodes.each(function (node) {
                    $(node.get('el')).remove();
                });
                if (typeof this.options.performances != 'undefined') {
                    this.options.performances.eventHandler = false;
                }
            },
            removeNodeElements: function () {
                this.model.nodes.each(function (node) {
                    node.unset('el');
                });
            },
            nodeClicked: function (e) {
                var node = new Node({
                    name: $(e.target).data('name'),
                    start_time: 0,
                    duration: 1
                });

                this.model.nodes.add(node);
            },
            modelChanged: function () {
                this.ui.performanceName.val(this.model.get('name'));
            },
            initDraggable: function (el, options) {
                if (!this.options.readonly)
                    $(el).draggable(_.extend({
                        helper: 'clone',
                        appendTo: 'body',
                        revert: 'invalid',
                        delay: 100,
                        snap: '.app-timeline-nodes',
                        snapMode: 'inner',
                        zIndex: 1000
                    }, options));
            },
            initResizable: function (el) {
                var self = this,
                    handle = $('<span>').addClass('ui-resizable-handle ui-resizable-e ui-icon ui-icon-gripsmall-diagonal-se');

                $(el).append(handle).resizable({
                    handles: 'w, e',
                    resize: function () {
                        var node = self.model.nodes.get({cid: $(this).data('cid')});
                        node.set('duration', Math.round($(this).outerWidth() / self.config.pxPerSec * 100) / 100);
                    }
                });
            },
            createNodeEl: function (node) {
                if (node.get('el')) return;
                var self = this,
                    el = $('.app-node[data-name="' + node.get('name') + '"]', this.ui.nodesContainer).clone()
                        .removeClass('hidden').get(0);

                this.initDraggable(el, {
                    start: function () {
                        // hide original when showing clone
                        $(this).hide();
                    },
                    stop: function () {
                        // show original when hiding clone
                        $(this).show();
                    }
                });
                if (_.contains(['emotion', 'interaction', 'expression'], node.get('name')) && !this.options.readonly) {
                    this.initResizable(el);
                }

                // show node settings on click
                $(el).attr('data-cid', node.cid).on('click', function () {
                    self.toggleNodeSettings(node);
                }).css({position: 'absolute'});

                node.set('el', el);
                node.on('change', this.updateNode, this);

                this.updateNode(node);
            },
            /**
             * Updates node element
             *
             * @param node Node
             */
            updateNode: function (node) {
                if (this.isDestroyed || ! node.get('el')) {
                    node.off('change', this.updateNode, this);
                    return;
                }

                var width = node.get('duration') * this.config.pxPerSec;

                $(node.get('el')).stop().css({
                    left: node.get('start_time') * this.config.pxPerSec,
                    width: width
                });

                if (node.hasChanged('duration') || node.hasChanged('start_time'))
                    this.arrangeNodes();

                if (node.get('text'))
                    node.get('el').childNodes[0].nodeValue = node.get('text');
                else if (node.get('emotion'))
                    node.get('el').childNodes[0].nodeValue = node.get('emotion');
                else if (node.get('gesture'))
                    node.get('el').childNodes[0].nodeValue = node.get('gesture');
                else if (node.get('expression'))
                    node.get('el').childNodes[0].nodeValue = node.get('expression');
                else if (node.get('animation'))
                    node.get('el').childNodes[0].nodeValue = node.get('animation');
            },
            addNode: function (node) {
                this.arrangeNodes();
                this.toggleNodeSettings(node);
            },
            toggleNodeSettings: function (node) {
                if (this.options.readonly) {
                    return;
                }
                var self = this,
                    showSettings = !this.nodeView || this.nodeView.model != node;

                this.ui.timelineContainer.find('.app-node').removeClass('active');
                if (showSettings)
                    $(node.get('el')).addClass('active');

                this.ui.nodeSettings.slideUp(function () {
                    // show settings if no settings are shown or if shown settings are for a different node
                    if (showSettings) {
                        self.nodeView = new NodeView({model: node});
                        self.nodeView.render();
                        self.ui.nodeSettings.html(self.nodeView.el).hide().slideDown();
                    } else {
                        self.hideNodeSettings();
                    }
                });
            },
            hideNodeSettings: function () {
                var self = this,
                    destroy = function () {
                        self.ui.timelineContainer.find('.app-node').removeClass('active');
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
                    available = false,
                    nodes = this.model.nodes;

                nodes.each(function (node) {
                    self.createNodeEl(node);

                    var begin = node.get('start_time'),
                        end = begin + node.get('duration');

                    $('.app-timeline-nodes', self.ui.timelineContainer).each(function () {
                        available = true;

                        $('.app-node', this).each(function () {
                            var model = nodes.findWhere({'el': this});

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

                        if (available) $(this).append(node.get('el'));
                        return !available;
                    });

                    if (!available) {
                        self.addTimeline();
                        $('.app-timeline-nodes:last-child').append(node.get('el'));
                    }
                });

                // remove empty timelines
                $('.app-timeline-nodes', this.el).filter(':empty').remove();

                // add an empty timeline
                this.addTimeline();
                this.updateTimelineWidth();
            },
            addTimeline: function () {
                this.ui.timelineContainer.append($('<div>').addClass('app-timeline-nodes'));
            },
            getTimelineWidth: function () {
                return this.model.getDuration() * this.config.pxPerSec;
            },
            updateTimelineWidth: function () {
                var width = this.getTimelineWidth(),
                    containerWidth = this.ui.timelineContainer.width(),
                    scaleWidth = Math.max(width, containerWidth),
                    scale = d3.scale.linear().domain([0, scaleWidth / this.config.pxPerSec]).range([0, scaleWidth]);

                // update axis
                this.ui.timeAxis.html('').width(scaleWidth);
                d3.select(this.ui.timeAxis.get(0)).call(d3.svg.axis().scale(scale).orient("bottom"));

                if (width < containerWidth || !containerWidth)
                    width = '100%';

                this.ui.timelineContainer.find('.app-timeline-nodes').each(function () {
                    $(this).css('width', width);
                });

                this.ui.scrollContainer.perfectScrollbar('update');
            },
            setPerformanceName: function () {
                this.model.set('name', this.ui.performanceName.val());
            },
            savePerformances: function () {
                var self = this,
                    path = '';

                if (this.performances && this.performances.currentPath)
                    path = this.performances.currentPath;

                this.model.save({path: this.model.get('path') || path}, {
                    success: function (model) {
                        if (self.performances) self.performances.add(model);
                        self.options.readonly = false;
                        self.ui.deleteButton.fadeIn();
                        self.ui.clearButton.fadeIn();
                        self.ui.nodesContainer.fadeIn();

                        if (model.get('error')) {
                            App.Utilities.showPopover(self.ui.saveButton, model.get('error'));
                            model.unset('error');
                        } else
                            App.Utilities.showPopover(self.ui.saveButton, 'Saved');
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
                var left = parseInt(this.ui.runIndicator.css('left'));
                if (!$.isNumeric(time))
                    time = parseInt(left) / this.config.pxPerSec;

                var step = 1. / App.getOption('fps'),
                    frameCount = parseInt(time / step);

                this.ui.frameCount.html(frameCount);
                this.ui.timeIndicator.html((frameCount * step).toFixed(2));
                this.ui.scrollContainer.scrollLeft(Math.max(0, left - this.ui.scrollContainer.innerWidth() * 0.1));
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
                this.model.nodes.reset();
                this.hideNodeSettings();
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
