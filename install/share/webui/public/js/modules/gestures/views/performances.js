define(["marionette", "./performance", 'entities/performance_collection'],
    function (Marionette, PerformanceView, PerformanceCollection) {
        return Marionette.CollectionView.extend({
            childView: PerformanceView,
            initialize: function (options) {
                if (!options.collection) {
                    this.collection = new PerformanceCollection();
                    this.collection.fetch();
                }
            }
        });
    });
