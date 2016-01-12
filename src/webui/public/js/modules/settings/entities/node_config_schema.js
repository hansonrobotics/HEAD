define(['backbone', 'lib/api', 'jquery'], function (Backbone, api, $) {
    return Backbone.Model.extend({
        initialize: function (node_name) {
            this.node_name = node_name;
        },
        sync: function (method, collection, options) {
            if (method == 'read') {
                api.services.get_node_description.callService({node: this.node_name}, function (response) {
                    var schema = collection.getSchemaFromDescription(JSON.parse(response.description));
                    options.success && options.success(schema);
                }, function (error) {
                    options.error && options.error(error);
                });
            }
        },
        getSchemaFromDescription: function (description) {
            var properties = {};

            $.each(description, function (i, param) {
                var property = {title: param.description};

                switch (param.type) {
                    case 'bool':
                        property.type = 'boolean';
                        property.format = 'checkbox';
                        break;
                    case 'int':
                        property.type = 'integer';
                        break;
                    case 'double':
                        property.type = 'number';
                        break;
                    case 'string':
                        property.type = 'string';
                        break;
                }

                if ($.isNumeric(param.min)) property.minimum = param.min;
                if ($.isNumeric(param.max)) property.maximum = param.max;

                properties[param.name] = property;
            });

            return {
                title: this.node_name + ' settings',
                type: 'object',
                properties: properties
            };
        }
    });
});
