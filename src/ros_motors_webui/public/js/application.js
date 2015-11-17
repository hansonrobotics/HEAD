define(['backbone', 'marionette', 'lib/ros', 'modules/layout/layout', 'lib/api'],
    function (Backbone, Marionette, ros, LayoutView, api) {
        var Application = new Marionette.Application(),
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
            // Enable for the whole app - blenderMode is the only mode for APAC demo
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
        // Make sure only monitor is loaded before connection is made
        Backbone.Router.prototype.execute = function (callback, args) {
            if (checkConnection())
                return;
            if (callback) callback.apply(this, args);
        };
        ros.connect(function () {
            require([
                    'modules/animations/animations_app',
                    'modules/expressions/expressions_app',
                    'modules/motors/motors_app',
                    'modules/gestures/gestures_app',
                    'modules/performances/performances_app',
                    'modules/interaction/interaction_app',
                    'modules/monitor/router',
                    'modules/status/status_app'],
                function () {
                    Application.start();
                });
        });

        return Application;
    });
