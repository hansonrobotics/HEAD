define(['marionette', './controller'], function (Marionette, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'admin/settings': 'settings'
        }
    });

    new Router({controller: controller});
});