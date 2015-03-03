define(["ros_ui", "tpl!./templates/expression.tpl"], function (UI, template) {
    UI.module("Motors.View", function (View, RosUI, Backbone, Marionette, $, _) {
        View.Expression = Marionette.ItemView.extend({
            template: template,
            modelEvents: {
                'change': 'modelChanged'
            },
            collectionEvents: {
                'change': 'collectionChanged'
            },
            ui: {
                button: 'button'
            },
            events: {
                'click @ui.button': 'select'
            },
            initialize: function (options) {
                this.expressionsView = options.expressionsView;
            },
            modelChanged: function () {
                var name = this.model.get('name');
                this.ui.button.text(name);

                if (name == '')
                    this.$el.hide();
                else
                    this.$el.show();
            },
            collectionChanged: function () {
                if (this.active())
                    this.model.set('motor_positions', this.collection.getMotorPositions());
            },
            select: function () {
                this.expressionsView.expressionButtonClicked(this);
                this.active(true);

                var self = this,
                    motorPositions = this.model.get('motor_positions'),
                    motorNames = _.keys(motorPositions);

                self.collection.each(function (motor) {
                    var name = motor.get('name');

                    if (_.indexOf(motorNames, name) != -1) {
                        motor.setValue(motorPositions[name]);
                        motor.selected(true);
                    } else {
                        motor.selected(false);
                    }
                });
            },
            active: function (val) {
                if (typeof val == 'undefined')
                    return this.ui.button.hasClass('active');
                else if (val) {
                    $('.expression-button').removeClass('active');
                    return this.ui.button.addClass('active');
                } else {
                    return this.ui.button.removeClass('active');
                }
            }
        });
    });

    return UI.Motors.View.Expression;
});
