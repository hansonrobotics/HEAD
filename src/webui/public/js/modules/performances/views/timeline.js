define(['application', 'tpl!./templates/timeline.tpl', 'jquery-ui', '../entities/node'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Timeline = Marionette.ItemView.extend({
            template: template,
            className: 'app-timeline-container',
            ui: {
                checkbox: '.app-timeline-checkbox input',
                nodes: '.app-timeline-nodes'
            }
        });
    });
});
