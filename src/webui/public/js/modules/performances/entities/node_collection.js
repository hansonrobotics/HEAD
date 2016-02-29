define(['backbone', './node'], function (Backbone, Node) {
    return Backbone.Collection.extend({
        model: Node,
        comparator: 'start_time'
    });
});