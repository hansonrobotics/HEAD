define(["app", "./motor"], function (UI, motorView) {
    UI.module("Motors.View", function (View, RosUI, Backbone, Marionette, $, _) {
        View.Motors = Marionette.CollectionView.extend({
            childView: motorView
        });
    });

    return UI.Motors.View.Motors;
});
