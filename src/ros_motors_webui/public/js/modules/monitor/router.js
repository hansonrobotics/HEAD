define(['application', './controller'], function (App, controller) {
    var Router = Marionette.AppRouter.extend({
        'appRoutes': {
            'admin/monitor': 'system',
            'admin/monitor/motors': 'motors',
            'admin/monitor/messages': 'messages',
            'admin/monitor/logs': 'logs',
            'admin/monitor/processes': 'processes',
            'admin/monitor/speed': 'speed',
            'admin/monitor/system': 'system'
        }
    });

    new Router({controller: controller});
});
