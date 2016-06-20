define(['backbone', './performance', 'lib/api'], function (Backbone, Performance, api) {
    return Backbone.Collection.extend({
        eventHandler: false,
        model: Performance,
        comparator: function (model) {
            return model.get('name').toLowerCase();
        },
        url: '/performances/' + api.config.robot,
        initialize: function () {
            // Subscribing to performance events
            var self = this;
            api.subPerformanceEvents(function (msg) {
                if (self.eventHandler) {
                    self.eventHandler(msg);
                }
            });
        }
    });
});
