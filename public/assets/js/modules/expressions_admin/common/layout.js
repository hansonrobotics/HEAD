define(['application', 'tpl!./templates/layout.tpl'], function (App, template) {
    App.module("Motors.Common", function (Common, App, Backbone, Marionette, $, _) {
        Common.Layout = Marionette.LayoutView.extend({
            template: template,

            regions: {
                motors: ".app-motors",
                expressions: ".app-expressions"
            }
        });
    });

    return App.Motors.Common.Layout;
});