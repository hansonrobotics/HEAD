define(['backbone', './node', 'supermodel'], function (Backbone, Node, Supermodel) {
    return Backbone.Collection.extend({
        model: function(attrs, options) {
            return Node.create(attrs, options);
        },
        comparator: 'start_time',
        initialize: function () {
            this.on('reset', function (col, opts) {
                _.each(opts.previousModels, function (model) {
                    model.removeEl();
                });
            });
        },
        _isModel: function(model) {
            return model instanceof Backbone.Model || model instanceof Supermodel.Model;
        },
    });
});
