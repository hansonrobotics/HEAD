define(['application', './controller'], function (App, controller) {
    App.module('Performances', function (Performances, app, Backbone, Marionette, $, _) {
        Performances.Router = Marionette.AppRouter.extend({
            'appRoutes': {
                'performances': 'performances',
                'attention': 'attention_regions'
            }
        });

        Performances.on('start', function () {
            new Performances.Router({controller: controller});
        });
    });
});
