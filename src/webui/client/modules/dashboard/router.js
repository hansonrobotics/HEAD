define(['application', './controller', 'marionette'], function (App, controller, Marionette) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'dashboard': 'demo'
        }
    });

    return new Router({controller: controller});
});
