define(['marionette', 'application', './controller'], function (Marionette, App, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'performances': 'performances',
            'attention': 'attention_regions'
        }
    });

    new Router({controller: controller});
});
