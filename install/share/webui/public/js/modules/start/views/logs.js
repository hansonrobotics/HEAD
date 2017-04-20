define(["marionette", "./log"],
    function (Marionette, logView) {
        return Marionette.CollectionView.extend({
            initialize: function (options) {
                this.listenTo(this.collection, 'reset', this.refresh);
            },
            childView: logView,
            onDestroy: function () {
                clearInterval(this.logInterval);
            },
            onRender: function () {
                var self = this;
                clearInterval(this.logInterval);
                this.logInterval = setInterval(function(){ self.collection.fetch();}, 2000)
            }
        });
    });
