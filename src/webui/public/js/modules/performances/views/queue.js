define(['marionette', 'backbone', 'tpl!./templates/queue.tpl', './timelines', 'underscore'],
    function (Marionette, Backbone, template, TimelinesView, _) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                queue: '.app-performance-queue',
                performances: '.app-performance-queue .app-performance',
                performanceTemplate: '.app-performance-template',
                clearButton: '.app-clear',
                emptyNotice: '.app-empty-notice'
            },
            events: {
                'click @ui.clearButton': 'clear'
            },
            queue: [],
            initialize: function (options) {
                this.mergeOptions(options, ['sequence', 'performances', 'layoutView']);
            },
            onAttach: function () {
                var self = this;

                $(this.ui.queue).sortable({
                    axis: "y",
                    handle: ".app-drag-handle",
                    placeholder: "ui-state-highlight",
                    stop: function () {
                        self.updateTimeline();
                    }
                });

                if (this.sequence instanceof Array && this.performances) {
                    _.each(this.sequence, function (id) {
                        var model = self.performances.get(id);
                        if (model) self.addPerformance(model);
                    });
                }
            },
            addPerformance: function (performance, skipTimelineUpdate) {
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
                    self.updateTimeline();
                });

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
                // Adding multiple performances at time makes it faster if timeline updated only once
                skipTimelineUpdate = skipTimelineUpdate || false;
                if (!skipTimelineUpdate)
                    this.updateTimeline();
            },
            updateTimeline: function () {
                this.timelinesView = this._showTimeline({sequence: this._getPerformanceIds(), readonly: true});
            },
            removePerformance: function (performance) {
                var self = this;

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
            clear: function () {
                this.stop();

                var self = this;
                _.each(this.queue, function (item) {
                    self.removePerformance(item.model);
                });

                this.updateTimeline();
                this.ui.clearButton.blur();
            },
            _getPerformanceIds: function () {
                var self = this,
                    ids = [];

                $('ul .app-performance:visible', this.el).each(function () {
                    var el = this,
                        index = _.findIndex(self.queue, function (item) {
                            return item && el == item.el;
                        });

                    if (index != -1) ids.push(self.queue[index].model.id);
                });

                return ids;
            },
            _showTimeline: function (options) {
                var timelinesView = new TimelinesView(_.extend({
                    performances: this.options.performances
                }, options));

                // show configuration UI
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
