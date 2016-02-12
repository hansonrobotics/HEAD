define(['backbone', './emotion', 'lib/api'], function (Backbone, Emotion, api) {
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
