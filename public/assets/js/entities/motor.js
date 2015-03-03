define(['../ros_ui', '../lib/api', '../lib/utilities'], function (UI, api, utilities) {
    UI.module('Entities', function (Entities, UI, Backbone, Marionette, $, _) {
        Entities.Motor = Backbone.Model.extend({
            initialize: function () {
                this.set('value', this.get('default'));
            },
            // converts given attribute value to radians
            getMinDeg: function () {
                return Math.ceil(utilities.radToDeg(this.get('min')));
            },
            getMaxDeg: function () {
                return Math.floor(utilities.radToDeg(this.get('max')));
            },
            getDefaultDeg: function () {
                return Math.round(utilities.radToDeg(this.get('default')));
            },
            setMinDeg: function (deg) {
                this.set('min', utilities.degToRad(deg));
            },
            setMaxDeg: function (deg) {
                this.set('max', utilities.degToRad(deg));
            },
            setDefaultDeg: function (deg) {
                this.set('default', utilities.degToRad(deg));
            },
            getValueDeg: function () {
                return Math.round(utilities.radToDeg(this.get('value')));
            },
            setValueDeg: function (deg) {
                this.setValue(utilities.degToRad(deg));
            },
            setValue: function (value) {
                this.set('value', value);

                if (this.get('name') == "neck_roll") {
                    api.pointHead({roll: utilities.degToRad(this.get('value'))});
                } else {
                    api.sendMotorCommand(this.toJSON(), utilities.degToRad(this.get('value')));
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
            }
        });
        Entities.MotorCollection = Backbone.Collection.extend({
            model: Entities.Motor,
            comparator: 'order_no',
            sync: function () {
                $.ajax("/" + api.config.robot + "/motors/update", {
                    data: JSON.stringify(this.toJSON()),
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
            }
        });
    });
});
