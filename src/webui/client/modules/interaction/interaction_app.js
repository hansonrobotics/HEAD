define(['application', './controller'], function (App, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'interactions': 'index'
        }
    });
    new Router({controller: controller});
});
