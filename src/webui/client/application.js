define(['backbone', 'marionette', 'modules/layout/layout', 'lib/ros', 'lib/api', 'jquery-ui', 'jquery-ui-touch-punch',
        'bootstrap'],
    function(Backbone, Marionette, LayoutView, ros, api) {
        $.ajaxSetup({
            contentType: 'application/json; charset=utf-8',
            dataType: 'json'
        })

        var Application = Marionette.Application.extend({
                region: '#app-content',

                onStart: function() {
                    app.LayoutInstance = new LayoutView({fluid: true})

                    // add layout to page body
                    app.showView(app.LayoutInstance)

                    ros.connect(function() {
                        if (Backbone.history)
                            Backbone.history.start({pushState: false})
                    })
                },
                language: 'en',
                Utilities: {
                    showPopover: function(el, label, placement) {
                        placement = placement || 'right'
                        $(el).popover({
                            content: label,
                            trigger: 'manual focus',
                            container: 'body',
                            placement: placement
                        }).on('remove', function() {
                            // destroy popover if element is destroyed
                            $(el).popover('destroy')
                        }).popover('show')

                        // destroy popover after 2 seconds
                        setTimeout(function() {
                            $(el).popover('destroy')
                        }, 2000)
                    }
                }
            }),
            app = new Application({
                fps: 48
            }),
            checkConnection = function() {
                // Allow the monitor to be open if not connected
                if (Backbone.history.getHash().match(/admin\/monitor/)) {
                    return false
                }
                if (!api.ros.connected) {
                    Backbone.history.navigate('admin/monitor', {trigger: true})
                    return true
                }
                return false
            }

        app.lastUrl = null
        Backbone.Router.prototype.execute = function(callback, args) {
            // Make sure only monitor is loaded before connection is made
            if (checkConnection())
                return

            // performs a custom check on each request allowing to perform actions before the new route is triggered
            // check if changeCheck function is defined, accepts a function with two arguments: success and cancel
            // callbacks in that order
            if (typeof app.changeCheck === 'function' && !app.skipChangeCheck) {
                if (!this.changeCheckVisible) {
                    let revertUrl = app.lastUrl
                    app.changeCheckVisible = true
                    app.changeCheck(function() {
                        app.changeCheckVisible = false
                        app.lastUrl = Backbone.history.getHash()
                        if (callback) callback.apply(this, args)
                    }, function() {
                        app.changeCheckVisible = false
                        // need this because of the # symbol at the beginning of url which prevents from changing url
                        // without triggering a route
                        app.skipChangeCheck = true
                        Backbone.history.navigate('#' + revertUrl, {trigger: false})
                        app.lastUrl = revertUrl
                    })
                }

                return false
            } else if (callback) {
                app.skipChangeCheck = false
                app.lastUrl = Backbone.history.getHash()
                callback.apply(this, args)
            }
        }

        return app
    })
