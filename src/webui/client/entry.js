require(['eventemitter2', 'application', 'jquery-ui', 'jquery-ui-touch-punch', 'bootstrap', 'application', 'modules/puppeteering/router',
        'modules/animations/animations_app',
        'modules/expressions/expressions_app',
        'modules/motors/motors_app',
        'modules/gestures/gestures_app',
        'modules/performances/performances_app',
        'modules/interaction/interaction_app',
        'modules/monitor/router',
        'modules/start/router',
        'modules/settings/router'],
    function (EventEmitter2, app) {
        $.ajaxSetup({
            contentType: 'application/json; charset=utf-8',
            dataType: 'json'
        });

        app.start();
    });
