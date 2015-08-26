define(['application', 'tpl!./templates/timelines.tpl', './timeline'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Timelines = Marionette.CompositeView.extend({
            template: template,
            childView: App.Performances.Views.Timeline,
            childViewContainer: '.app-timelines',
            ui: {
                addButton: '.app-add-timeline-button',
                removeButton: '.app-remove-timeline-button',
                selectAll: '.app-timeline-select-all input'
            },
            events: {
                'click @ui.addButton': 'add',
                'click @ui.removeButton': 'removeSelected',
                'click @ui.selectAll': 'selectAll'
            },
            add: function () {
                this.collection.add(new App.Performances.Entities.Timeline({}));
            },
            removeSelected: function () {
                var self = this,
                    any  = false;

                // remove selected
                this.children.each(function (view) {
                    if (view.selected()) {
                        self.collection.remove(view.model);
                        any = true;
                    }
                });

                // remove last timeline if none selected
                if (! any && self.collection.length > 0) {
                    self.collection.pop();
                }
            },
            selectAll: function () {
                var checked = this.ui.selectAll.prop('checked');
                this.children.each(function (view) {
                    view.selected(checked);
                });
            }
        });
    });
});
