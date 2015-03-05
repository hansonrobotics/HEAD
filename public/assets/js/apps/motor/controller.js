define(["ros_ui", "lib/api", "./common/layout", "./show/motors", './expression/expressions',
        'entities/motor', 'entities/expression'],
    function (RosUi, api, LayoutView, MotorsView, ExpressionsView) {
        return {
            show: function () {
                var motors = new RosUi.Entities.MotorCollection(),
                    expressions = new RosUi.Entities.ExpressionCollection(),
                    layoutView = new LayoutView(),
                    motorsView = new MotorsView({
                        collection: motors
                    }),
                    expressionsView = new ExpressionsView({
                        collection: expressions,
                        motors: motors,
                        controller: this
                    });

                expressions.fetch();

                $('#app-page-motors').html(layoutView.render().el);
                layoutView.getRegion('motors').show(motorsView);
                layoutView.getRegion('expressions').show(expressionsView);
                self = this;
                api.getMotorsFromFile(function (data) {
                    motors.add(data.motors);
                    self.loadPololuMotors(motors);
                });
            },
            loadPololuMotors: function (collection) {
                api.getPololuMotorTopics(function (topics) {
                    _.each(topics, function (topic) {
                        for (var i = 0; i < 24; i++) {
                            var unique = true,
                                newMotor = new RosUi.Entities.Motor({
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

                            _.each(collection.models, function (motor) {
                                if (motor.get('motor_id') == newMotor.get('motor_id') &&
                                    motor.get(topic) == newMotor.get('topic')) {
                                    unique = false;
                                }
                            });

                            if (unique) collection.add(newMotor);
                        }
                    });
                });
            }
        };
    });