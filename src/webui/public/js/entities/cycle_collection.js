define(['backbone', 'lib/api', 'underscore'], function (Backbone, api, _) {
    return Backbone.Collection.extend({
        sync: function (method, collection, options) {
            if (method == 'read')
                api.getAvailableSomaStates(function (somaState) {
                    var data = [];
                    _.each(somaState, function (name) {
                        data.push({name: name});
                    });
                    options.success(data);
                });
        }
    });
});
