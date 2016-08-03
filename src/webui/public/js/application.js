define(['backbone', 'marionette', 'lib/ros', 'modules/layout/layout', 'lib/api', 'robot_config'],
    function (Backbone, Marionette, ros, LayoutView, api, RobotConfig) {
        var Application = new Marionette.Application({
                fps: 48
            }),
            checkConnection = function () {
                // Allow the monitor to be open if not connected
                if (Backbone.history.getHash().match(/admin\/monitor/)) {
                    return false;
                }
                if (api.ros.connected == false) {
                    Backbone.history.navigate('admin/monitor', {trigger: true})
                    return true;
                }
                return false;
            };
        Application.on("start", function () {
            if (Backbone.history)
                Backbone.history.start();
            console.log('started');
        });
        // store layout instance in App.Layout.Instance
        Application.LayoutInstance = new LayoutView({fluid: true});

        // add layout to page body
        $('body').prepend(Application.LayoutInstance.render().el);

        Application.Utilities = {
            showPopover: function (el, label, placement) {
                placement = placement || 'right';
                $(el).popover({
                    content: label,
                    trigger: 'manual focus',
                    container: 'body',
                    placement: placement
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
        Application.language = 'en';

        if (RobotConfig.mode == 'start') {
            // Lightweight application for startup
            require([
                    'modules/start/router'
                ],
                function () {
                    Application.start();
                });
        } else {
            // Make sure only monitor is loaded before connection is made
            Backbone.Router.prototype.execute = function (callback, args) {
                if (checkConnection())
                    return;
                if (callback) callback.apply(this, args);
            };
            ros.connect(function () {
                require([
                        'modules/puppeteering/router',
                        'modules/animations/animations_app',
                        'modules/expressions/expressions_app',
                        'modules/motors/motors_app',
                        'modules/gestures/gestures_app',
                        'modules/performances/performances_app',
                        'modules/interaction/interaction_app',
                        'modules/monitor/router',
                        'modules/start/router',
                        'modules/settings/router',
                        'modules/status/status_app'],
                    function () {
                        Application.start();
                    });
            });
        }
        return Application;
    });
