define(['application', 'tpl!./templates/performances.tpl', './performance'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Performances = Marionette.CompositeView.extend({
            template: template,
            childView: App.Performances.Views.Performance,
            childViewContainer: '.app-performances',
            ui: {
                newButton: '.app-new-performance-button'
            },
            events: {
                'click @ui.newButton': 'addNew'
            },
            addNew: function () {
                var performance = new App.Performances.Entities.Performance({name: 'New performance'});
                this.collection.add(performance);
                Views.trigger('performance:add', performance);
            },
            _insertAfter: function(childView) {
                // insert all performance buttons before new button
                this.ui.newButton.before(childView.el);
            }
        });
    });
});
