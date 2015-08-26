define(['application', 'tpl!./templates/performances.tpl', './performance'], function (App, template) {
    App.module('Performances.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Performances = Marionette.CompositeView.extend({
            template: template,
            childView: App.Performances.Views.Performance,
            childViewContainer: '.app-performances'
        });
    });
});
