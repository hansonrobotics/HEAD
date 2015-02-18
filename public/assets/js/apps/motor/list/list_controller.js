define(['app', './../layout/layout', './motors', 'main/lib/ros', 'entities/motor'],
    function (UI, LayoutView, MotorsView, ros) {
        UI.module('MotorApp.List', function (List, ContactManager, Backbone, Marionette, $, _) {
            List.Controller = {
                list: function () {
                    var layout = new LayoutView(),
                        motorsCollection = new UI.Entities.MotorCollection(),
                        motorView = new MotorsView({
                            collection: motorsCollection
                        });

                    ros.loadMotorConfig(function (motors) {
                        motorsCollection.add(motors);
                    });

                    $('#app-page-motors').html(layout.render().$el);
                    layout.getRegion('motorRegion').show(motorView);
                }
            };
        });

        return UI.MotorApp.List.Controller;
    });
