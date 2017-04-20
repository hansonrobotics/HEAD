define(['marionette', './controller'], function (Marionette, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'puppeteering': 'index'
        }
    });

    new Router({controller: controller});
});