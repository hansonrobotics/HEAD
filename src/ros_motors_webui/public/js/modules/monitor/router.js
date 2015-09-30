define(['application', './controller'], function (App, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'monitor': 'motors',
            'monitor/motors': 'motors',
            'monitor/messages': 'messages',
            'monitor/logs': 'logs',
            'monitor/processes': 'processes',
            'monitor/speed': 'speed',
            'monitor/system': 'system'
        }
    });

    App.on('start', function () {
        new Router({controller: controller});
    });
});
