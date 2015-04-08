define(['application', 'tpl!./templates/layout.tpl'], function (App, template) {
    App.module("Motors.Views", function (Common, App, Backbone, Marionette, $, _) {
        Common.Layout = Marionette.LayoutView.extend({
            template: template,

            regions: {
                motors: ".app-motors",
            }
        });
    });

    return App.module('Motors.Views.Layout');
});