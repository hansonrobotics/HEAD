define(["application", "./gesture"], function (App, GestureView) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Gestures = Marionette.CollectionView.extend({
            childView: GestureView
        });
    });

    return App.module('Gestures.Views').Gestures;
});
