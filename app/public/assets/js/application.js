define(['backbone', 'marionette', 'lib/ros', 'modules/layout/layout'],
    function (Backbone, Marionette, ros, LayoutView) {
        var Application = new Marionette.Application();

        Application.on("start", function () {
            if (Backbone.history)
                Backbone.history.start();
        });

        // store layout instance in App.Layout.Instance
        Application.LayoutInstance = new LayoutView();

        // add layout to page body
        $('body').prepend(Application.LayoutInstance.render().el);

        ros.connect(function () {
            require([
                    'modules/animations/animations_app',
                    'modules/status/status_app',
                    'modules/expressions/expressions_app',
                    'modules/expressions_admin/motor_app',
                    'modules/gestures/gestures_app',
                    'modules/interaction/interaction_app'],
                function () {
                    Application.start();
                });
        });

        return Application;
    });
