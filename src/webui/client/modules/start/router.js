define(['marionette', './controller'], function (Marionette, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'system': 'system',
            'logs': 'logs',
            '': 'system'
        }
    });
    new Router({controller: controller});
});
