define(["application", "tpl!./templates/status.tpl"], function (App, template) {
    App.module("Status.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Status = Marionette.LayoutView.extend({
            template: template,
            ui: {
                indicators: '#app-status-indicators'
            }
        });
    });

    return App.module('Status.Views').Status;
});
