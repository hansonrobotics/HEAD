define(['application', './controller'], function (App, controller) {
    App.module('Motors', function (Motors, app, Backbone, Marionette, $, _) {
        Motors.Router = Marionette.AppRouter.extend({
            'appRoutes': {
                'motors': 'index',
                'admin/motors': 'admin_index'
            }
        });

        Motors.on('start', function () {
            new Motors.Router({controller: controller});
        });
    });
});
