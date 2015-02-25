define(['../../../ros_ui', './motors', '../../../lib/api', '../../../lib/ros', 'entities/motor'],
    function (UI, MotorsView, api, ros) {
        UI.module('MotorApp.List', function (List, ContactManager, Backbone, Marionette, $, _) {
            List.Controller = {
                list: function () {
                    var motorsCollection = new UI.Entities.MotorCollection(),
                        motorView = new MotorsView({
                            collection: motorsCollection
                        });

                    // load motors from config
                    api.getMotorsConfig(function (motors) {
                        motorsCollection.add(motors);
                        // load motors from pololu board
                        api.getPololuMotorTopics(function (topics) {
                            _.each(topics, function (topic) {
                                for (var i = 0; i < 24; i++) {
                                    var unique = true,
                                        newMotor = new UI.Entities.Motor({
                                            name: i,
                                            topic: topic,
                                            min: -Math.PI / 2,
                                            max: Math.PI / 2,
                                            default: 0,
                                            editable: true,
                                            labelleft: '',
                                            labelright: ''
                                        });

                                    _.each(motorsCollection.models, function (motor) {
                                        if (motor.get('name') == newMotor.get('name') &&
                                            motor.get(topic) == newMotor.get('topic')) {
                                            unique = false;
                                        }
                                    });

                                    if (unique) motorsCollection.add(newMotor);
                                }
                            });
                        });
                    });

                    $('#app-page-motors').html(motorView.render().$el);
                }
            }
        });

        return UI.MotorApp.List.Controller;
    });
