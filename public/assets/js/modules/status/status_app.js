define(['application', './controller'], function (App, controller) {
    App.module('Status', function (Status, app, Backbone, Marionette, $, _) {
        Status.Router = Marionette.AppRouter.extend({
            'appRoutes': {
                'status': 'index'
            }
        });

        Status.on('start', function () {
            new Status.Router({controller: controller});
        });
    });
});
