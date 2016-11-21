define(['application', 'lib/api', './motor'], function (App, api, Motor) {
    return Backbone.Collection.extend({
        model: Backbone.Model,
        comparator: 'order',
        fetch: function () {
            var collection = this;

            $.ajax('/expressions/' + api.config.robot, {
                cache: false,
                dataType: 'json',
                success: function (data) {
                    var expressions = data.expressions;

                    _.each(expressions, function (expression, i) {
                        _.each(expression, function (motorPositions, name) {
                            var model = new Motor({
                                name: name,
                                motor_positions: motorPositions,
                                order: i
                            });

                            collection.add(model);
                        });
                    })
                }
            });
        },
        sync: function (successCallback, errorCallback) {
            var expressions = [];

            this.each(function (model) {
                var expression = {};
                var name = model.get('name');

                if (name != '') {
                    expression[name] = model.get('motor_positions');
                    expressions.push(expression);
                }
            });

            $.ajax("/expressions/update/" + api.config.robot, {
                method: 'POST',
                data: JSON.stringify({expressions: expressions}),
                type: 'POST',
                dataType: "json",
                success: function () {
                    api.setExpressionsParam(expressions);

                    if (typeof successCallback == 'function')
                        successCallback();
                },
                error: function () {
                    if (typeof errorCallback == 'function')
                        errorCallback();
                }
            });
        }
    });
});
