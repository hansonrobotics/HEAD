define(['marionette', 'backbone', './templates/queue.tpl', './timelines', 'underscore', 'lib/regions/fade_in',
    '../entities/performance'], function (Marionette, Backbone, template, TimelinesView, _, FadeInRegion, Performance) {
    return Marionette.View.extend({
        template: template,
        ui: {
            container: '.app-queue-container',
            queue: '.app-performance-queue',
            performances: '.app-performance-queue .app-performance',
            performanceTemplate: '.app-performance-template',
            clearButton: '.app-clear',
            emptyNotice: '.app-empty-notice'
        },
        events: {
            'click @ui.clearButton': 'clearClick'
        },
        regions: {
            timeline: {
                el: '.app-timeline-region',
                regionClass: FadeInRegion
            }
        },
        queue: [],
        initialize: function (options) {
            this.mergeOptions(options, ['sequence', 'performances', 'layoutView', 'readonly', 'hidden']);
        },
        onRender: function () {
            let self = this;

            if (this.hidden)  {
                this.ui.container.hide();
            } else {
                $(this.ui.queue).sortable({
                    axis: "y",
                    handle: ".app-drag-handle",
                    placeholder: "ui-state-highlight",
                    stop: function () {
                        self.updateTimeline();
                    }
                });
            }


            if (this.options.sequence)
                this.showSequence(this.options.sequence);

            if (this.readonly) {
                this.runningPerformances = new Performance();
                this.runningPerformances.enableSync(function (performances) {
                    self.showSequence(_.map(performances, 'id'), true);
                });
            }
        },
        onDestroy: function () {
            if (this.readonly) this.runningPerformances.disableSync();
        },
        showSequence: function (sequence, skipTimelineUpdate) {
            let self = this;

            if (!_.isEqual(this._getPerformanceIds(), sequence)) {
                this.clearQueue();
                if (sequence instanceof Array && this.performances) {
                    _.each(sequence, function (id) {
                        let model = self.performances.get(id);
                        if (model) self.addPerformance(model, true);
                    });
                }

                if (!skipTimelineUpdate) this.updateTimeline();
            }
        },
        showCurrent: function () {
            let self = this,
                current = new Performance();

            current.fetchCurrent({
                success: function (response) {
                    self._showTimeline({
                        model: current,
                        running: response['running'],
                        paused: response['paused'],
                        current_time: response['current_time'],
                        readonly: true
                    });

                    self.showSequence(_.map(response['performances'], 'id'), true);
                }
            });
        },
        editPerformance: function (performance, options) {
            this._showTimeline({
                model: performance,
                readonly: false
            });
        },
        addPerformance: function (performance, skipTimelineUpdate) {
            let self = this,
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
                self.updateTimeline();
            });

            if (this.readonly)
                $('.app-edit', el).hide();
            else
                $('.app-edit', el).click(function () {
                    self.stop();
                    self._showTimeline({model: performance});
                });

            performance.on('change', function () {
                self._updateItem(item);
            });

            performance.on('destroy', function () {
                self._removeItem(item);
                self.updateTimeline();
            });

            if (!skipTimelineUpdate) this.updateTimeline();
        },
        updateTimeline: function (options) {
            this.timelinesView = this._showTimeline(_.extend({
                sequence: this._getPerformanceIds(),
                readonly: true
            }, options || {}));
        },
        removePerformance: function (performance) {
            let self = this;

            _.each(this.queue, function (item) {
                if (item.model == performance)
                    self._removeItem(item)
            });
        },
        stop: function () {
            if (this.timelinesView) {
                this.timelinesView.destroy();
                this.timelinesView = null;
            }
        },
        clearClick: function () {
            this.stop();
            this.clearQueue();
            this.updateTimeline();
            this.ui.clearButton.blur();
        },
        clearQueue: function () {
            let self = this;
            _.each(this.queue, function (item) {
                self._removeItem(item);
            });
        },
        _getPerformanceIds: function () {
            let self = this,
                ids = [];

            if (this.hidden) {
                this.queue.forEach(function (item) {
                    ids.push(item.model.id)
                });
            } else {
                $('ul .app-performance:visible', this.el).each(function () {
                    let el = this,
                        index = _.findIndex(self.queue, function (item) {
                            return item && el == item.el;
                        });

                    if (index != -1) ids.push(self.queue[index].model.id);
                });
            }

            return ids;
        },
        _showTimeline: function (options) {
            let self = this;
            this.timelinesView = new TimelinesView(_.extend({
                performances: this.options.performances
            }, options));

            this.timelinesView.on('close', function () {
                if (!self.isDestroyed()) self.updateTimeline();
            });

            // show configuration UI
            this.getRegion('timeline').show(this.timelinesView);
            return this.timelinesView;
        },
        _removeItem: function (item) {
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
