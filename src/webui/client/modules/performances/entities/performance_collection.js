define(['backbone', './performance', 'lib/api', 'backbone.naturalsort'], function (Backbone, Performance, api) {
    return Backbone.Collection.extend({
        eventHandler: false,
        model: Performance,
        sortType: 'natural',
        comparator: function (model) {
            return (model.get('name') || '').toLowerCase();
        },
        url: function () {
            return '/performances/' + api.config.robot;
        },
        initialize: function () {
            // Subscribing to performance events
            let self = this;
            api.subPerformanceEvents(function (msg) {
                if (self.eventHandler) {
                    self.eventHandler(msg);
                }
            });
        }
    });
});
