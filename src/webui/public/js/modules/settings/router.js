define(['marionette', './controller'], function (Marionette, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'admin/settings': 'robot',
            'admin/settings/node': 'robot'
        }
    });

    new Router({controller: controller});
});