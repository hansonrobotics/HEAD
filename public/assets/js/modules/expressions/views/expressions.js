define(["application", "tpl!./templates/expressions.tpl"], function (App, template) {
    App.module("Expressions.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Expressions = Marionette.LayoutView.extend({
            template: template,
            ui: {
            }
        });
    });

    return App.module('Expressions.Views').Expressions;
});
