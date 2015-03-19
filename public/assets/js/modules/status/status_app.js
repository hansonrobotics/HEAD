define(['application', './controller'], function (App, controller) {
    App.module('Status', function (Status, app, Backbone, Marionette, $, _) {
        Status.Router = Marionette.AppRouter.extend({
            'appRoutes': {
                '': 'index',
                'status': 'index',
                'admin': 'admin_index'
            }
        });

        Status.on('start', function () {
            new Status.Router({controller: controller});
        });
    });
});
