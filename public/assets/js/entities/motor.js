define(['application', 'lib/api', 'lib/utilities'], function (App, api, utilities) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Motor = Backbone.Model.extend({
            initialize: function () {
                // set value to default
                this.set('value', this.get('default'));

                // not selected by default
                this.set('selected', false);

                // send motor command when updating value
                this.on('change', this.checkUpdatedValue())
            },
            checkUpdatedValue: function () {
                var self = this;

                return function () {
                    if (this.previous('value') != this.get('value')) {
                        api._sendMotorCommand(self.toJSON(), self.get('value'));
                    }
                };
            },
            selected: function (selected) {
                if (typeof selected == 'undefined') {
                    // return status if arg undefined
                    return this.get('selected');
                } else {
                    // set status otherwise
                    this.set('selected', !!selected);
                }
            },
            getDegrees: function (attribute) {
                return Math.round(utilities.radToDeg(this.get(attribute)));
            },
            setDegrees: function (attribute, value) {
                return this.set(attribute, utilities.degToRad(value));
            },
            // Sets value 0-1 mapped to min and max.
            setRelativeVal: function (attribute, value) {
                var min = this.get('min');
                var max = this.get('max');
                var v = (min + (max - min) * value);
                this.set(attribute, v);
            },
            getRelativeVal: function (attribute) {
                var min = this.get('min');
                var max = this.get('max');
                var v = this.get(attribute);
                return Math.min(1, Math.max(0, (v - min) / (max - min)));
            }
        });
        Entities.MotorCollection = Backbone.Collection.extend({
            model: Entities.Motor,
            comparator: 'order_no',
            sync: function () {
                var data = [];

                this.each(function (motor) {
                    if (motor.get('labelleft')) {
                        motor = motor.toJSON();
                        delete motor['selected'];
                        delete motor['value'];

                        data.push(motor);
                    }
                });

                var param = new ROSLIB.Param({ros: api.ros, name: '/' + api.config.robot + '/motors'});
                param.set(data);

                $.ajax("/api/motors/update" + "/" + api.config.robot, {
                    data: JSON.stringify(data),
                    type: 'POST',
                    dataType: "json",
                    success: function () {
                    }
                });
            },
            getRelativePositions: function () {
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
