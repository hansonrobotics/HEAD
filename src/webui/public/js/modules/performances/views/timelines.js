define(['application', 'marionette', 'tpl!./templates/timelines.tpl', 'd3', 'bootbox', './node',
        '../entities/node', 'underscore', 'jquery', '../entities/performance', 'lib/regions/fade_in', 'lib/speech_recognition',
        'lib/api', 'lib/extensions/animate_auto', 'jquery-ui', 'scrollbar'],
    function (App, Marionette, template, d3, bootbox, NodeView, Node, _, $, Performance, FadeInRegion, speechRecognition, api) {
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
                nodeSettings: '.app-node-settings-container',
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
                closeButton: '.app-close-button'
            },
            regions: {
                nodeSettings: {
                    el: '.app-node-settings-container',
                    regionClass: FadeInRegion
                }
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
                'click @ui.closeButton': 'close',
                'click @ui.loopButton': 'loop'
            },
            modelEvents: {
                'change': 'modelChanged'
            },
            initialize: function (options) {
                var self = this,
                    loadOptions = {
                        success: function () {
                            var handler = function () {
                                if (self.isDestroyed)
                                    self.model.nodes.off('change add remove', handler);
                                else
                                    self.model.loadNodes();
                            };

                            self.model.nodes.on('change add remove', handler);
                        }
                    };

                this.mergeOptions(options, ['performances']);

                if (options.sequence instanceof Array) {
                    this.model = new Performance();
                    this.model.loadSequence(options.sequence, loadOptions);
                } else if (this.model) {
                    if (this.model.id && this.model.nodes.isEmpty()) {
                        this.model.load(loadOptions);
                    } else
                        this.model.loadNodes(loadOptions);
                } else {
                    this.model = new Performance();
                    this.model.fetchCurrent(loadOptions);
                }
            },
            childViewOptions: function () {
                return {performance: this.model, config: this.config};
            },
            close: function () {
                this.trigger('close');
            },
            onAttach: function () {
                var self = this;

                this.ui.scrollContainer.droppable({
                    accept: '[data-node-name], [data-node-id]',
                    tolerance: 'touch',
                    drop: function (e, ui) {
                        var el = $(ui.helper),
                            id = el.data('node-id'),
                            node = Node.all().get(id),
                            startTime = Math.round(
                                    ($(this).scrollLeft() + ui.offset.left - $(this).offset().left) / self.config.pxPerSec * 100) / 100;

                        if (id && node) {
                            node.set('start_time', startTime);
                            self.model.nodes.add(node);
                        } else {
                            node = self.model.nodes.add({
                                name: el.data('node-name'),
                                start_time: startTime,
                                duration: 1
                            });
                            self.showNodeSettings(node);
                        }
                    }
                });

                if (this.options.readonly) {
                    this.ui.nodesContainer.hide();
                    this.ui.closeButton.hide();
                } else {
                    this.nodeView = new NodeView({collection: this.model.nodes});
                    this.getRegion('nodeSettings').show(this.nodeView);
                }

                this.modelChanged();
                this.ui.scrollContainer.perfectScrollbar();

                // Performance event handler
                if (typeof this.options.performances != 'undefined')
                    this.options.performances.eventHandler = function (msg) {
                        self.handleEvents(msg);
                    };

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
                var node = Node.create({
                    name: $(e.target).data('name'),
                    start_time: 0,
                    duration: 1
                });

                this.model.nodes.add(node);
            },
            modelChanged: function () {
                this.ui.performanceName.val(this.model.get('name'));
            },
            initResizable: function (el) {
                var self = this,
                    handle = $('<span>').addClass('ui-resizable-handle ui-resizable-e ui-icon ui-icon-gripsmall-diagonal-se');

                $(el).append(handle).resizable({
                    handles: 'w, e',
                    resize: function () {
                        var node = self.model.nodes.get({cid: $(this).data('node-id')});
                        node.set('duration', Math.round($(this).outerWidth() / self.config.pxPerSec * 100) / 100);
                    }
                });
            },
            createNodeEl: function (node) {
                var self = this,
                    el = $('<div>').addClass('app-node label')
                        .attr('data-node-name', node.get('name'))
                        .attr('data-node-id', node.cid)
                        .on('click', function () {
                            self.showNodeSettings(node);
                        });

                node.set('el', el.get(0));
                node.off('change', this.updateNode, this);
                node.on('change', this.updateNode, this);

                this.updateNode(node);

                if (!this.options.readonly) {
                    el.draggable({
                        helper: 'clone',
                        appendTo: 'body',
                        revert: 'invalid',
                        delay: 100,
                        snap: '.app-timeline-nodes',
                        snapMode: 'inner',
                        zIndex: 1000,
                        start: function () {
                            // hide original when showing clone
                            $(this).hide();
                        },
                        stop: function () {
                            $(this).show();
                        }
                    });

                    if (node.hasProperty('duration'))
                        this.initResizable(el);
                }
            },
            /**
             * Updates node element
             *
             * @param node Node
             */
            updateNode: function (node) {
                if (this.isDestroyed || !node.get('el')) {
                    node.off('change', this.updateNode, this);
                } else {
                    var el = $(node.get('el'));
                    el.stop().css({
                        left: node.get('start_time') * this.config.pxPerSec,
                        width: node.get('duration') * this.config.pxPerSec
                    }).attr('data-node-name', node.get('name'));

                    if (node.hasChanged('duration') || node.hasChanged('start_time'))
                        this.arrangeNodes();

                    if (el.is(':empty'))
                        el.html(node.getTitle());
                    else
                        el.get(0).childNodes[0].nodeValue = node.getTitle();
                }
            },
            addNode: function (node) {
                this.arrangeNodes();
            },
            showNodeSettings: function (node) {
                if (this.options.readonly) return;
                this.ui.timelineContainer.find('.app-node').removeClass('active');
                if (node.get('el')) $(node.get('el')).addClass('active');
                this.nodeView.showSettings(node);
            },
            arrangeNodes: function () {
                var self = this,
                    available = false,
                    nodes = this.model.nodes;

                nodes.each(function (node) {
                    var begin = node.get('start_time'),
                        end = begin + node.get('duration');

                    if (!node.get('el'))
                        self.createNodeEl(node);

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
                } else if (e.event == 'idle')
                    this.stopIndicator();
                else if (e.event == 'running')
                    this.startIndicator(e.time, duration);
                else if (e.event == 'resume')
                    this.startIndicator(e.time, duration);
                else if (e.event == 'chat')
                    this.enableChat();
                else if (e.event == 'chat_end')
                    this.disableChat();
            },
            enableChat: function () {
                var self = this;
                this.chatEnabled = true;
                if (speechRecognition) {
                    this.speechRecognition = speechRecognition.getInstance();
                    speechRecognition.continuous = true;

                    this.speechRecognition.onresult = function (event) {
                        var mostConfidentResult = speechRecognition.getMostConfidentResult(event.results);
                        if (mostConfidentResult) api.topics.listen_node_input.publish({data: mostConfidentResult.transcript});
                    };

                    this.speechRecognition.onend = function () {
                        if (self.chatEnabled)
                            self.enableChat();
                    };

                    this.speechRecognition.start();
                }
            },
            disableChat: function () {
                this.chatEnabled = false;
                if (this.speechRecognition) {
                    this.speechRecognition.abort();
                    this.speechRecognition = null;
                }
            }
        });
    });
