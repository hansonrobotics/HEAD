define(["application", "./performance"], function (App, PerformanceView) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Performances = Marionette.CollectionView.extend({
            childView: PerformanceView
        });
    });

    return App.module('Gestures.Views').Performances;
});
