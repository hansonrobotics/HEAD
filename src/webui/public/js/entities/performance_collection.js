define(['backbone', 'lib/api', 'underscore'], function (Backbone, api, _) {
    return Backbone.Collection.extend({
        sync: function (method, collection, options) {
            if (method == 'read')
                api.getAvailableScripts(function (scripts) {
                    console.log('here');
                    var data = [];
                    _.each(scripts, function (name) {
                        data.push({name: name});
                    });
                    options.success(data);
                });
        }
    });
});
