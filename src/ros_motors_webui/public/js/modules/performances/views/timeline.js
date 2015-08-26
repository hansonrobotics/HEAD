define(['application', 'tpl!./templates/timeline.tpl'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Timeline = Marionette.ItemView.extend({
            template: template,
            className: 'app-timeline',
            ui: {
                checkbox: '.app-timeline-checkbox input'
            },
            selected: function (value) {
                if (typeof value == 'undefined')
                    return this.ui.checkbox.prop('checked');
                else {
                    this.ui.checkbox.prop('checked', value);
                }
            }
        });
    });
});
