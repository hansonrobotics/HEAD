define(['application', './controller'], function (App, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'motors': 'public_index',
            'admin/motors': 'admin_index'
        }
    });

    new Router({controller: controller});
})
