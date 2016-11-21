define(['application', './controller'], function (App, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'admin/animations': 'admin_index'
        }
    });

    new Router({controller: controller});
})
