define(["application", "./cycle"], function (App, CycleView) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Cycles = Marionette.CollectionView.extend({
            childView: CycleView
        });
    });

    return App.module('Gestures.Views').Cycles;
});
