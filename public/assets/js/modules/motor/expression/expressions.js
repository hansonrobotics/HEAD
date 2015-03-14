define(["application", "./expression", 'tpl!./templates/expressions.tpl'],
    function (App, expressionView, template) {
        App.module("Motors.View", function (View, App, Backbone, Marionette, $, _) {
            View.Expressions = Marionette.CompositeView.extend({
                template: template,
                childViewContainer: '.app-expressions',
                childView: expressionView,
                ui: {
                    nameField: '#app-expression-name',
                    addButton: '.app-create-expression',
                    saveButton: '.app-save-expressions'
                },
                events: {
                    'keyup @ui.nameField': 'changeExpressionsName',
                    'click @ui.addButton': 'addExpressions',
                    'click @ui.saveButton': 'updateExpressions'
                },
                initialize: function (options) {
                    this.motors = options.motors;
                    this.childViewOptions = {
                        collection: options.motors,
                        expressionsView: this
                    };
                },
                addExpressions: function () {
                    var expression = new App.Entities.Expression({
                        name: 'NewExpression',
                        motor_positions: this.motors.getRelativePositions()
                    });
                    this.collection.add(expression);
                },
                updateExpressions: function () {
                    this.collection.sync();
                },
                expressionButtonClicked: function (view) {
                    App.vent.trigger('motors:selection:set', true);

                    if (this.last_clicked != view) {
                        this.last_clicked = view;
                        this.ui.nameField.val(view.model.get('name'));
                    }
                },
                changeExpressionsName: function () {
                    if (typeof this.last_clicked != 'undefined')
                        this.last_clicked.model.set('name', this.ui.nameField.val());
                }
            });
        });

        return App.Motors.View.Expressions;
    });
