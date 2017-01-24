define(['application', './controller'], function (App, controller) {
    let Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'admin/animations': 'admin_index'
        }
    });

    new Router({controller: controller});
});
