define(["marionette", "./cycle", 'entities/cycle_collection'],
    function (Marionette, CycleView, CycleCollection) {
        return Marionette.CollectionView.extend({
            childView: CycleView,
            initialize: function (options) {
                if (!options.collection) {
                    this.collection = new CycleCollection();
                    this.collection.fetch();
                }
            }
        });
    });
