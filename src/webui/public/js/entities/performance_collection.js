define(['backbone', 'lib/api'], function (Backbone, api) {
    return Backbone.Collection.extend({
        added: false,
        sync: function (method, collection, options) {
            var self = this;
            api.getAvailableScripts(function (scripts) {
                if (!self.added) {
                    self.added = true;
                    var data = [];
                    _.each(scripts, function (name) {
                        data.push({name: name});
                    });
                    options.success(data);
                }
            });
        }
    });
});
