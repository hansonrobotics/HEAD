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
            return this.constructor.getSchemaFromDesc(description, this.node_name)
        }
    },
    {
        getSchemaFromDesc: function (description, node_name) {
            var properties = {};

            $.each(description, function (i, param) {
                var property = {title: param['description']};

                try {
                    param['edit_method'] = JSON.parse(param['edit_method'].split("'").join('"'));
                } catch (e) {
                    param['edit_method'] = null;
                }

                switch (param.type) {
                    case 'bool':
                        property.type = 'boolean';
                        property.format = 'checkbox';
                        break;
                    case 'int':
                        property.type = 'slider';
                        property.step = 1;
                        break;
                    case 'double':
                        property.type = 'slider';
                        property.step = 0.01;
                        break;
                    case 'str':
                        property.type = 'string';
                        break;
                }

                if (param['edit_method'] && param['edit_method']['enum'].length > 0) {
                    property.enum = [];

                    $.each(param['edit_method']['enum'], function (i, attr) {
                        property.enum.push(attr.value);
                    });
                }

                if ($.isNumeric(param.min)) property.minimum = param.min;
                if ($.isNumeric(param.max)) property.maximum = param.max;

                properties[param.name] = property;
            });

            return {
                title: node_name + ' settings',
                type: 'object',
                properties: properties
            };
        }
    });
});
