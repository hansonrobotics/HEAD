define(['application', './controller'], function (App, controller) {
    App.module('Expressions', function (Expressions, app, Backbone, Marionette, $, _) {
        Expressions.Router = Marionette.AppRouter.extend({
            'appRoutes': {
                '': 'index',
                'expressions': 'index'
            }
        });

        Expressions.on('start', function () {
            new Expressions.Router({controller: controller});
        });
    });
});
