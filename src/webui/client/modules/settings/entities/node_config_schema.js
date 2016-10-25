define(['backbone', 'lib/api', './node_config'], function (Backbone, api, NodeConfig) {
    return Backbone.Model.extend({
        initialize: function (node_name) {
            this.node_name = node_name;
        },
        sync: function (method, self, options) {
            if (method == 'read') {
                var node_schema = new Promise(function (resolve, reject) {
                    var node = new NodeConfig(self.node_name);
                    node.fetch({
                        success: function () {
                            resolve(JSON.parse(node.get('node_schema') || '{}'))
                        },
                        error: function (err) {
                            reject(err);
                        }
                    })
                }), desc_schema = new Promise(function (resolve, reject) {
                    api.services.get_node_description.callService({node: self.node_name}, function (response) {
                        var schema = self.getSchemaFromDescription(JSON.parse(response.description));
                        resolve(schema);
                    }, function (error) {
                        reject(error);
                    });
                });

                Promise.all([desc_schema, node_schema]).then(function (data) {
                    var properties = {};
                    for (let prop of data) {
                        properties = _.extend(properties, prop);
                    }
                    options.success && options.success({
                        title: self.node_name + ' settings',
                        type: 'object',
                        properties: properties
                    });
                    // returned data is in arguments[0], arguments[1], ... arguments[n]
                    // you can process it here
                }, function (err) {
                    options.error && options.error(err);
                });
            }
        },
        getSchemaFromDescription: function (description) {
            var properties = {};

            $.each(description, function (i, param) {
                // hide node schema field
                if (param.name == 'node_schema') return;

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

            return properties;
        }
    });
});
