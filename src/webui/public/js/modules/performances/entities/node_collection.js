define(['backbone', './node'], function (Backbone, Node) {
    return Backbone.Collection.extend({
        model: Node,
        comparator: 'start_time',
        initialize: function () {
            this.on('reset', function (col, opts) {
                _.each(opts.previousModels, function (model) {
                    model.removeEl();
                });
            });
        }
    });
});