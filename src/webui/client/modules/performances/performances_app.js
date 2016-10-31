define(['application', './controller'], function (App, controller) {
    App.module('Performances', function (Performances, app, Backbone, Marionette, $, _) {
        Performances.Router = Marionette.AppRouter.extend({
            'appRoutes': {
                'editor': 'performances'
            }
        });

        Performances.on('start', function () {
            new Performances.Router({controller: controller});
        });
    });
});
