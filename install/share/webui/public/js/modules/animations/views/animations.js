define(['application', './animation'], function (App, animationView) {
    App.module('Animations.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Animations = Marionette.CollectionView.extend({
            childView: animationView
        });
    });

    return App.module('Animations.Views').Animations;
});