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
            })

        app.state = new Backbone.Model()

        return app
    })
