define(['marionette', './controller'], function (Marionette, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            '': 'index',
            'puppeteering': 'index'
        }
    });

    new Router({controller: controller});
});