define(["application", "./expression", './templates/expressions.tpl'],
    function (App, expressionView, template) {
        App.module("Expressions.Admin.Views", function (View, App, Backbone, Marionette, $, _) {
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
                    this.mergeOptions(options, ['motors']);
                    this.childViewOptions = {
                        collection: this.motors,
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
                    var self = this;

                    this.collection.sync(function () {
                        App.Utilities.showPopover(self.ui.saveButton, 'Saved')
                    }, function () {
                        App.Utilities.showPopover(self.ui.saveButton, 'Error saving expressions')
                    });
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

        return App.module('Expressions.Admin.Views.Expressions');
    });
