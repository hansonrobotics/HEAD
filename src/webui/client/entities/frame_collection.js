define([], function () {
    return Backbone.Collection.extend({
        comparator: 'order_no'
    });
});
