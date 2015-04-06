define(["application", "tpl!./templates/interaction.tpl"], function (App, template) {
    App.module("Interaction.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Interaction = Marionette.LayoutView.extend({
            template: template,
            ui: {
                indicators: '#app-status-indicators'
            }
        });
    });

    return App.module('Interaction.Views').Interaction;
});
