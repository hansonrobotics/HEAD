define(["marionette", "./log"],
    function (Marionette, logView) {
        return Marionette.CollectionView.extend({
            childView: logView
        });
    });
