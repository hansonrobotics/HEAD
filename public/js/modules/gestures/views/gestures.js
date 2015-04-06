define(["application", "tpl!./templates/gestures.tpl"], function (App, template) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Gestures = Marionette.LayoutView.extend({
            template: template
        });
    });

    return App.module('Gestures.Views').Gestures;
});
