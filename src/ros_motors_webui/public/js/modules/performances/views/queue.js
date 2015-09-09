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
                clearButton: '.app-clear',
                emptyNotice: '.app-empty-notice'
            },
            events: {
                'click @ui.runButton': 'run',
                'click @ui.pauseButton': 'pause',
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
                    self.pause();

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
                    if (item.model == performance) {
                        self.removeItem(item)

                    }
                });

                if (this.queue.length == 0)
                    this.ui.emptyNotice.slideDown();
            },
            removeItem: function (item) {
                $(item.el).slideUp();
                this.queue = _.without(this.queue, item);
            },
            updateItem: function (item) {
                $('.app-name', item.el).html(item.model.get('name'));
                $('.app-duration', item.el).html(item.model.getDuration());
            },
            run: function () {
                if (this.queue.length == 0 || this.running) return;
                this.running = true;

                var self = this,
                    performanceUnion = this.getPerformanceUnion(),
                    duration = performanceUnion.getDuration(),
                    timelinesView = this.showTimeline(performanceUnion);

                var run = function () {
                    timelinesView.runPerformance(function () {
                        setTimeout(function () {
                            if (self.running)
                                run();
                            else
                                timelinesView.destroy();
                        }, duration * 1000);
                    });
                };

                run();
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
            pause: function () {
                this.running = false;
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
