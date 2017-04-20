define(['backbone', './emotion', 'lib/api', 'underscore'], function (Backbone, Emotion, api, _) {
    return Backbone.Collection.extend({
        model: Emotion,
        sync: function (method, collection, options) {
            if (method == 'read')
                api.getAvailableEmotionStates(function (emotions) {
                    var data = [];
                    _.each(emotions, function (name) {
                        data.push({name: name});
                    });
                    options.success(data);
                });
        }
    });
});
