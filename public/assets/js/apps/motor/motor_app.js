define(["ros_ui", "./common/layout", "./show/motors", 'entities/motor'], function (RosUi, LayoutView, MotorsView) {
    RosUi.module("MotorApp", function (MotorApp, RosUi, Backbone, Marionette, $, _) {
        MotorApp.on("start", function () {
            var motorsCollection = new RosUi.Entities.MotorCollection(),
                layoutView = new LayoutView(),
                motorsView = new MotorsView({
                    collection: motorsCollection
                });
            $('#app-page-motors').html(layoutView.render().el);

            layoutView.getRegion('motors').show(motorsView);

            // load motors from config
            require('lib/api').getMotorsConfig(function (motors) {
                motorsCollection.add(motors);

                // load motors from pololu board
                api.getPololuMotorTopics(function (topics) {
                    _.each(topics, function (topic) {
                        for (var i = 0; i < 24; i++) {
                            var unique = true,
                                newMotor = new RosUi.Entities.Motor({
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
        });
    });

    return RosUi.MotorApp;
});
