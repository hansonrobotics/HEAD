define(['application', './animation'], function (App, animationView) {
    return Marionette.CollectionView.extend({
        childView: animationView
    });
});