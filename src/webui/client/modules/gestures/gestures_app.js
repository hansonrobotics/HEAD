define(['application', './controller'], function (App, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            '': 'index',
            'gestures': 'index'
        }
    });

    new Router({controller: controller});
});
