define(['application', './templates/layout.tpl'], function (App, template) {
    App.module("Motors.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Layout = Marionette.LayoutView.extend({
            template: template,

            regions: {
                motors: ".app-motors",
            }
        });
    });

    return App.module('Motors.Views.Layout');
});