define(["application", "./emotion"], function (App, EmotionView) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Emotions = Marionette.CollectionView.extend({
            childView: EmotionView
        });
    });

    return App.module('Gestures.Views').Emotions;
});
