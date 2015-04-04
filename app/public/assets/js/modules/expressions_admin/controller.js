define(["application", "lib/api", "./common/layout", "./show/motors", './expression/expressions',
        'entities/motor', 'entities/expression'],
    function (App, api, LayoutView, MotorsView, ExpressionsView) {
        return {
            index: function () {
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
                api.pointHead();

                this.motorsCollection = new App.Entities.MotorCollection(),
                    this.layoutView = new LayoutView(),
                    this.motorsView = new MotorsView({
                        collection: this.motorsCollection,
                        disable_edit: true
                    });

                App.LayoutInstance.setTitle('Motors');
                App.LayoutInstance.getRegion('content').show(this.layoutView);
                App.LayoutInstance.showNav();

                this.layoutView.getRegion('motors').show(this.motorsView);

                var self = this;
                api.getMotorsFromParam(function (data) {
                    self.motorsCollection.add(data);
                    self.loadPololuMotors(self.motorsCollection);
                });
            },
            admin_index: function () {
                this.index();
                App.LayoutInstance.showAdminNav();

                var expressions = new App.Entities.ExpressionCollection(),
                    expressionsView = new ExpressionsView({
                        collection: expressions,
                        motors: this.motorsCollection,
                        controller: this
                    });

                expressions.fetch();
                this.layoutView.getRegion('expressions').show(expressionsView);
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