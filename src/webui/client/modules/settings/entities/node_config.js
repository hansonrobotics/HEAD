define(['backbone', 'roslib', 'lib/api', 'underscore'], function(Backbone, ROSLIB, api, _) {
    return Backbone.Model.extend({
        initialize: function(attrs, options) {
            this.node_name = options['node_name']
            this.readonly = options['readonly'] || false
        },
        sync: function(method, model, options) {
            let self = this

            if (method === 'read') {
                api.services.get_node_configuration.callService({node: this.node_name}, function(response) {
                    options.success && options.success(JSON.parse(response.configuration))
                }, function(error) {
                    options.error && options.error(error)
                })
            } else {
                if (!this.readonly)
                    _.each(model.changed, function(value, name) {
                        api.setDynParam(self.node_name, name, value)
                    })

                options.success && options.success(model.toJSON())
            }
        }
    })
})
