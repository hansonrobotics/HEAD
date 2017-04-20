define(['application', './controller'], function (App, controller) {
    App.module('Animations', function (Animations, app, Backbone, Marionette, $, _) {
        Animations.Router = Marionette.AppRouter.extend({
            'appRoutes': {
                'admin/animations': 'admin_index'
            }
        });

        Animations.on('start', function () {
            new Animations.Router({controller: controller});
        });
    });
});
