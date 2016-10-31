define(['marionette', './controller'], function (Marionette, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            '': 'index'
        }
    });

    new Router({controller: controller});
});