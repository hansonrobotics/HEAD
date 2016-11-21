define(['application', './animation'], function (App, animationView) {
    return Marionette.CollectionView.extend({
        childView: animationView,
        initialize: function (options) {
            this.mergeOptions(options, ['layout'])
        },
        childViewOptions: function () {
            return {layout: this.layout}
        }
    });
});