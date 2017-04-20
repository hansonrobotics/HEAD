define(['application', 'backbone', 'lib/api', 'underscore'], function (App, Backbone, api, _) {
    return Backbone.Collection.extend({
        sync: function (method, collection, options) {
            if (method == 'read')
                api.getAvailableGestures(function (gestures) {
                    var data = [];
                    _.each(gestures, function (name) {
                        data.push({name: name});
                    });
                    options.success(data);
                });
        }
    })
});
