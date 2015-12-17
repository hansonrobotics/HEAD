define(['backbone', 'roslib', 'lib/api', 'underscore'], function (Backbone, ROSLIB, api, _) {
    return Backbone.Model.extend({
        initialize: function (node_name) {
            this.node_name = node_name;
        },
        sync: function (method, model, options) {
            var self = this;

            if (method == 'read') {
                api.services.get_node_configuration.callService({node: this.node_name}, function (response) {
                    options.success && options.success(JSON.parse(response.configuration));
                }, function (error) {
                    options.error && options.error(error);
                });
            } else {
                _.each(model.changed, function (value, name) {
                    api.setDynParam(self.node_name, name, value);
                });

                options.success && options.success(model.toJSON());
            }
        }
    });
});
