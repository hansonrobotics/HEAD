define(['application', './controller'], function (App, controller) {
    App.module('Gestures', function (Gestures, App, Backbone, Marionette, $, _) {
        Gestures.Router = Marionette.AppRouter.extend({
            'appRoutes': {
                '': 'index',
                'gestures': 'index'
            }
        });

        Gestures.on('start', function () {
            new Gestures.Router({controller: controller});
        });
    });
});
