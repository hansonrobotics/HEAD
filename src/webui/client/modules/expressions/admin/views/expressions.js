define(["application", "./expression", './templates/expressions.tpl'],
    function (App, expressionView, template) {
        return Marionette.CompositeView.extend({
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
                var expression = new Backbone.Model({
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
                App.trigger('motors:selection:set', true);

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
