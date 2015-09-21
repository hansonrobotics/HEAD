define(['application', 'tpl!./templates/queue.tpl', './timelines'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Queue = Marionette.ItemView.extend({
            template: template,
            ui: {
                queue: '.app-performance-queue',
                performances: '.app-performance-queue .app-performance',
                performanceTemplate: '.app-performance-template',
                runButton: '.app-run',
                pauseButton: '.app-pause',
                stopButton: '.app-stop',
                clearButton: '.app-clear',
                emptyNotice: '.app-empty-notice'
            },
            events: {
                'click @ui.runButton': 'run',
                'click @ui.pauseButton': 'pause',
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
                this.updateItem(item);
                this.queue.push(item);
                this.ui.queue.append(el);

                $('.app-remove', el).click(function () {
                    self.removeItem(item);
                });

                $('.app-edit', el).click(function () {
                    self.stop();

                    var timelineView = self.showTimeline(performance);
                    timelineView.enableEdit();
                });

                performance.on('change', function () {
                    self.updateItem(item);
                });
            },
            removePerformance: function (performance) {
                var self = this;

                _.each(this.queue, function (item) {
                    if (item.model == performance)
                        self.removeItem(item)
                });
            },
            removeItem: function (item) {
                $(item.el).slideUp();
                this.queue = _.without(this.queue, item);

                if (this.queue.length == 0)
                    this.ui.emptyNotice.slideDown();
            },
            updateItem: function (item) {
                $('.app-name', item.el).html(item.model.get('name'));
                $('.app-duration', item.el).html(item.model.getDuration().toFixed(2));
            },
            run: function () {
                if (this.queue.length == 0) return;
                if (! this.timelinesView)
                    this.timelinesView = this.showTimeline(this.getPerformanceUnion());

                this.timelinesView.loop();
            },
            showTimeline: function (performance) {
                var timelinesView = new App.Performances.Views.Timelines({
                    collection: new Backbone.Collection(),
                    model: performance
                });

                // show configuration UI
                this.options.layoutView.getRegion('timeline').destroy();
                this.options.layoutView.getRegion('timeline').show(timelinesView);

                return timelinesView;
            },
            stop: function () {
                if (this.timelinesView) {
                    this.timelinesView.stop();
                    this.timelinesView.destroy();
                    this.timelinesView = null;
                }
            },
            pause: function () {
                if (this.timelinesView) this.timelinesView.pause();
            },
            getPerformanceUnion: function () {
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
            clear: function () {
                this.pause();

                var self = this;
                _.each(this.queue, function (item) {
                    self.removePerformance(item.model);
                });
            }
        });
    });
});
