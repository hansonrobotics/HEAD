define(['../ros_ui', '../lib/api', '../lib/utilities'], function (UI, api, utilities) {
    UI.module('Entities', function (Entities, UI, Backbone, Marionette, $, _) {
        Entities.Motor = Backbone.Model.extend({
            initialize: function () {
                // set value to default
                this.set('value', this.get('default'));

                // send motor command when updating value
                this.on('change', this.checkUpdatedValue)
            },
            checkUpdatedValue: function () {
                if (this.previous('value') != this.get('value')) {
                    api.sendMotorCommand(this.toJSON(), this.get('value'));
                }
            },
            selected: function (selected) {
                if (typeof selected == 'undefined') {
                    // return status if arg undefined
                    return (typeof this.motor_selected == 'undefined') ? false : this.motor_selected;
                } else {
                    // set status otherwise
                    this.motor_selected = selected;
                    this.trigger('change');
                }
            },
            getDegrees: function (attribute) {
                return Math.round(utilities.radToDeg(this.get(attribute)));
            },
            setDegrees: function (attribute, value) {
                return this.set(attribute, utilities.degToRad(value));
            },
            // Sets value 0-1 mapped to min and max.
            setRelativeVal: function(attribute, value){
                var min = this.get('min');
                var max = this.get('max');
                var v = (min+(max-min)*value);
                this.set(attribute, v);
            },
            getRelativeVal: function (attribute) {
                var min = this.get('min');
                var max = this.get('max');
                var v = this.get(attribute);
                return (v-min)/(max-min);
            }
        });
        Entities.MotorCollection = Backbone.Collection.extend({
            model: Entities.Motor,
            comparator: 'order_no',
            sync: function () {
                var data = [];

                this.each(function (motor) {
                    if (motor.get('labelleft'))
                        data.push(motor.toJSON());
                });

                $.ajax("/" + api.config.robot + "/motors/update", {
                    data: JSON.stringify(data),
                    type: 'POST',
                    dataType: "json"
                });
            },
            getMotorPositions: function () {
                var positions = {};

                this.each(function (motor) {
                    if (motor.selected())
                        positions[motor.get('name')] = motor.get('value');
                });

                return positions;
            },
            getRelativePositions: function() {
                var positions = {};

                this.each(function (motor) {
                    if (motor.selected())
                        positions[motor.get('name')] = motor.getRelativeVal('value');
                });
                return positions;                
            }
        });
    });
});
