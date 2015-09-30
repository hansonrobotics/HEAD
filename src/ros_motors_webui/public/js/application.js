define(['backbone', 'marionette', 'lib/ros', 'modules/layout/layout', 'lib/api'],
    function (Backbone, Marionette, ros, LayoutView, api) {
        var Application = new Marionette.Application();

        Application.on("start", function () {
            // Enable for the whole app - blenderMode is the only mode for APAC demo
            api.blenderMode.enable();

            if (Backbone.history)
                Backbone.history.start();
        });

        // store layout instance in App.Layout.Instance
        Application.LayoutInstance = new LayoutView();

        // add layout to page body
        $('body').prepend(Application.LayoutInstance.render().el);

        Application.Utilities = {
            showPopover: function (el, label) {
                $(el).popover({
                    content: label,
                    trigger: 'manual focus',
                    container: 'body'
                }).on('remove', function () {
                    // destroy popover if element is destroyed
                    $(el).popover('destroy');
                }).popover('show');

                // destroy popover after 2 seconds
                setTimeout(function () {
                    $(el).popover('destroy');
                }, 2000)
            }
        };

        ros.connect(function () {
            require([
                    'modules/animations/animations_app',
                    'modules/expressions/expressions_app',
                    'modules/motors/motors_app',
                    'modules/gestures/gestures_app',
                    'modules/performances/performances_app',
                    'modules/interaction/interaction_app',
                    'modules/monitor/router'
                ],
                function () {
                    Application.start();
                });
        });

        return Application;
    });
