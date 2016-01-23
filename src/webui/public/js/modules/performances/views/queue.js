define(['application', 'tpl!./templates/queue.tpl', './timelines'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Queue = Marionette.ItemView.extend({
            template: template,
            ui: {
                queue: '.app-performance-queue',
                performances: '.app-performance-queue .app-performance',
                performanceTemplate: '.app-performance-template',
                runButton: '.app-run',
                loopButton: '.app-loop-button',
                stopButton: '.app-stop',
                clearButton: '.app-clear',
                emptyNotice: '.app-empty-notice'
            },
            events: {
                'click @ui.runButton': 'run',
                'click @ui.loopButton': 'loop',
                'click @ui.stopButton': 'stop',
                'click @ui.clearButton': 'clear'
            },
            queue: [],
            onRender: function () {
                $(this.ui.queue).sortable({
                    axis: "y",
                    handle: ".app-drag-handle",
                    placeholder: "ui-state-highlight"
                });
            },
            addPerformance: function (performance) {
                var self = this,
                    el = $(this.ui.performanceTemplate).clone().removeClass('app-performance-template').get(0),
                    item = {
                        model: performance,
                        el: el
                    };

                this.ui.emptyNotice.slideUp();
                this._updateItem(item);
                this.queue.push(item);
                this.ui.queue.append(el);

                $('.app-remove', el).click(function () {
                    self._removeItem(item);
                });

                $('.app-edit', el).click(function () {
                    self.stop();
                    self._showTimeline(performance);
                });

                performance.on('change', function () {
                    self._updateItem(item);
                });
            },
            removePerformance: function (performance) {
                var self = this;

                _.each(this.queue, function (item) {
                    if (item.model == performance)
                        self._removeItem(item)
                });
            },
            run: function () {
                if (this.queue.length == 0) return;
                if (!this.timelinesView) {
                    this.performanceUnion = this._getPerformanceUnion();
                    this.timelinesView = this._showTimeline(this.performanceUnion);
                }

                this.timelinesView.run();
            },
            loop: function () {
                this.run();
                this.timelinesView.loop();
            },
            stop: function () {
                if (this.timelinesView) {
                    this.timelinesView.destroy();
                    this.timelinesView = null;
                }
            },
            clear: function () {
                this.stop();

                var self = this;
                _.each(this.queue, function (item) {
                    self.removePerformance(item.model);
                });
            },
            _getPerformanceUnion: function () {
                var self = this,
                    union = new App.Performances.Entities.Performance(),
                    startTime = 0;

                $('ul .app-performance', this.el).each(function () {
                    var el = this,
                        index = _.findIndex(self.queue, function (item) {
                            return item && el == item.el;
                        });

                    if (index != -1) {
                        var performance = self.queue[index].model;

                        performance.get('nodes').each(function (node) {
                            var clone = node.clone();
                            clone.set('start_time', clone.get('start_time') + startTime);
                            union.get('nodes').add(clone);
                        });

                        startTime += performance.getDuration();
                    }
                });

                return union;
            },
            _setHighlightTimeouts: function () {
                // clear any previous timeouts
                this._clearHighlightIntervals();

                // highlight active performances
                var self = this,
                    startTime = 0,
                    resumeTime = this.performanceUnion.getResumeTime(),
                    duration = this.performanceUnion.getDuration();

                // default to 0
                resumeTime = resumeTime ? resumeTime : 0;

                $.each(this.queue, function (i, item) {
                    var itemDuration = this.model.getDuration(),
                        timeoutTime;

                    if (startTime >= resumeTime)
                        timeoutTime = (startTime - resumeTime);
                    else
                        timeoutTime = duration - resumeTime + startTime;

                    item.highlightTimeout = setTimeout(function () {
                        self._highlightItem(item);

                        item.highlightInterval = setInterval(function () {
                            self._highlightItem(item);
                        }, duration * 1000);
                    }, timeoutTime * 1000);

                    startTime += itemDuration;
                });
            },
            _clearHighlightIntervals: function () {
                $.each(this.queue, function () {
                    clearTimeout(this.highlightTimeout);
                    clearInterval(this.highlightInterval);
                });
            },
            _removeHighlights: function () {
                $.each(this.queue, function () {
                    $('.app-status-indicator', this.el).removeClass('active');
                });
            },
            _highlightItem: function (item) {
                this._removeHighlights();
                $('.app-status-indicator', item.el).addClass('active');
            },
            _showTimeline: function (performance) {
                var timelinesView = new App.Performances.Views.Timelines({
                    collection: new Backbone.Collection(),
                    model: performance,
                    performances: this.options.performances
                });

                // show configuration UI
                this.options.layoutView.getRegion('timeline').destroy();
                this.options.layoutView.getRegion('timeline').show(timelinesView);

                return timelinesView;
            },
            _removeItem: function (item) {
                this.stop();

                $(item.el).slideUp();
                this.queue = _.without(this.queue, item);

                if (this.queue.length == 0)
                    this.ui.emptyNotice.slideDown();
            },
            _updateItem: function (item) {
                $('.app-name', item.el).html(item.model.get('name'));
                $('.app-duration', item.el).html(item.model.getDuration().toFixed(2));
            }
        });
    });
});
