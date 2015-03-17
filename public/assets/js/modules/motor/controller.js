define(["application", "lib/api", "./common/layout", "./show/motors", './expression/expressions',
        'entities/motor', 'entities/expression'],
    function (App, api, LayoutView, MotorsView, ExpressionsView) {
        return {
            show: function () {
                var motors = new App.Entities.MotorCollection(),
                    expressions = new App.Entities.ExpressionCollection(),
                    layoutView = new LayoutView(),
                    motorsView = new MotorsView({
                        collection: motors,
                        disable_edit: true
                    }),
                    expressionsView = new ExpressionsView({
                        collection: expressions,
                        motors: motors,
                        controller: this
                    });

                $('#app-page-motors').html(layoutView.render().el);

                layoutView.getRegion('motors').show(motorsView);

                App.getAdminEnabled(function(enabled) {
                    if (enabled) {
                        expressions.fetch();
                        layoutView.getRegion('expressions').show(expressionsView);
                    }
                });

                var self = this;
                api.getMotorsFromParam(function (data) {
                    motors.add(data);
                    self.loadPololuMotors(motors);
                });
            },
            loadPololuMotors: function (collection) {
                api.getPololuMotorTopics(function (topics) {
                    _.each(topics, function (topic) {
                        for (var i = 0; i < 24; i++) {
                            var unique = true,
                                newMotor = new App.Entities.Motor({
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