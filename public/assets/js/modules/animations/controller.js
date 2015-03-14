define(['jquery', 'application', './animations/animations', './animations/layout', '../motor/show/motors',
        'lib/api', 'entities/animation'],
    function ($, App, AnimationsView, LayoutView, MotorsView, api) {
        return {
            init: function () {
                var animations = new App.Entities.AnimationsCollection(),
                    motors = new App.Entities.MotorCollection(),
                    layout = new LayoutView(),
                    motorsView = new MotorsView({
                        collection: motors
                    }),
                    animationsView = new AnimationsView({
                        collection: animations
                    });

                // show views
                $('#app-page-animations').html(layout.render().el);
                layout.getRegion('animationButtons').show(animationsView);
                layout.getRegion('motors').show(motorsView);

                // load data
                animations.fetch();
                api.getMotorsFromFile(function (data) {
                    motors.add(data);
                });

                App.module('Animations.Views').on('animation_selected', function (button) {
                    console.log(button);
                })

            }
        };
    });
