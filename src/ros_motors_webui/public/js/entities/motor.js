define(['application', 'lib/api', 'lib/utilities'], function (App, api, utilities) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Motor = Backbone.Model.extend({
            initialize: function () {
                // set value to default
                this.set('value', this.get('default'));

                // not selected by default
                this.set('selected', false);

                // send motor command when updating value
                this.on('change', this.checkUpdatedValue());
            },
            checkUpdatedValue: function () {
                var self = this;

                return function () {
                    if (this.previous('value') != this.get('value')) {
                        api._sendMotorCommand(self.toJSON(), self.get('value'));
                    }
                };
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
            comparator: 'sort_no',
            initialize: function() {
                // clear error status interval
                this.on('destroy', this.clearErrorStatusInterval);
            },
            sync: function (successCallback, errorCallback) {
                var data = _.filter(this.toJSON(), function (motor) {
                    delete motor['selected'];
                    delete motor['editable'];
                    delete motor['value'];

                    return motor;
                });

                $.ajax("/motors/update/" + api.config.robot, {
                    data: JSON.stringify(data),
                    type: 'POST',
                    dataType: "json",
                    success: function () {
                        if (typeof successCallback == 'function')
                            successCallback();
                    },
                    error: function () {
                        if (typeof errorCallback == 'function')
                            errorCallback();
                    }
                });
            },
            fetchFromParam: function (callback) {
                var self = this;

                api.getMotorsFromParam(function (motors) {
                    self.add(motors);
                    self.loadPololuMotors();

                    if (typeof callback == 'function')
                        callback(motors);
                });
            },
            fetchFromFile: function (callback) {
                var self = this;

                $.ajax('/motors/get/' + api.config.robot, {
                    dataType: 'json',
                    success: function (response) {
                        self.add(response.motors);

                        if (typeof callback == 'function')
                            callback(response.motors);
                    }
                });
            },
            setErrorStatusInterval: function () {
                var self = this;

                this.clearErrorStatusInterval();
                this.checkErrorInterval = setInterval(function () {
                    $.ajax('/motors/status/' + api.config.robot, {
                        dataType: 'json',
                        success: function (response) {
                            _.each(response.motors, function (motor) {
                                var model = self.findWhere({name: motor['name']});
                                if (model)
                                    model.set('error', motor['error']);
                            });
                        }
                    });
                }, 1000);
            },
            clearErrorStatusInterval: function() {
                clearInterval(this.checkErrorInterval);
            },
            setDefaultValues: function () {
                this.each(function (motor) {
                    motor.set('value', motor.get('default'));
                })
            },
            getRelativePositions: function () {
                var positions = {};

                this.each(function (motor) {
                    if (motor.selected())
                        positions[motor.get('name')] = motor.getRelativeVal('value');
                });
                return positions;
            },
            loadPololuMotors: function () {
                var self = this;
                api.getPololuMotorTopics(function (topics) {
                    _.each(topics, function (topic) {
                        for (var i = 0; i < 24; i++) {
                            var unique = true,
                                newMotor = new Entities.Motor({
                                    name: i,
                                    motor_id: i,
                                    topic: topic,
                                    min: -Math.PI / 2,
                                    max: Math.PI / 2,
                                    default: 0,
                                    editable: true,
                                    labelleft: '',
                                    labelright: ''
                                });

                            _.each(self.models, function (motor) {
                                if (motor.get('motor_id') == newMotor.get('motor_id') &&
                                    motor.get(topic) == newMotor.get('topic')) {
                                    unique = false;
                                }
                            });

                            if (unique) self.add(newMotor);
                        }
                    });
                });
            }
        });
    });
});
