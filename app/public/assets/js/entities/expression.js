define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Expression = Backbone.Model.extend({});
        Entities.ExpressionCollection = Backbone.Collection.extend({
            model: Entities.Expression,
            comparator: 'order',
            fetch: function() {
                var collection = this;

                $.ajax('/expressions/' + api.config.robot, {
                    cache: false,
                    dataType: 'json',
                    success: function (data) {
                        var expressions = data.expressions;

                        _.each(expressions, function (expression, i) {
                            _.each(expression, function(motorPositions, name) {
                                var model = new Entities.Motor({
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
            sync: function () {
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
                    }
                });
            }
        });
    });
});
