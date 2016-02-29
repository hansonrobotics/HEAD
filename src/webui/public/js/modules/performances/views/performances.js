define(['marionette', 'tpl!./templates/performances.tpl', './performance', '../entities/performance'],
    function (Marionette, template, PerformanceView, Performance) {
        return Marionette.CompositeView.extend({
            template: template,
            childView: PerformanceView,
            childViewContainer: '.app-performances',
            ui: {
                newButton: '.app-new-performance-button'
            },
            events: {
                'click @ui.newButton': 'addNew'
            },
            addNew: function () {
                var performance = new Performance({name: 'New performance'});
                this.collection.add(performance);
                this.trigger('new', performance);
            },
            _insertAfter: function (childView) {
                var self = this;

                // add performance to the queue on click
                childView.on('click', function (data) {
                    self.options.queueView.addPerformance(data.model);
                });

                // insert all performance buttons before new button
                this.ui.newButton.before(childView.el);
            }
        });
    });
