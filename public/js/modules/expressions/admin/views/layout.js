define(['application', 'tpl!./templates/layout.tpl'], function (App, template) {
    App.module("Expressions.Admin.Views", function (Common, App, Backbone, Marionette, $, _) {
        Common.Layout = Marionette.LayoutView.extend({
            template: template,

            regions: {
                motors: ".app-motors",
                expressions: ".app-expressions"
            }
        });
    });

    return App.module('Expressions.Admin.Views.Layout');
});