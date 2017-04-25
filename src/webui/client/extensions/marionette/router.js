let app = require('application'),
    api = require('lib/api'),
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

Backbone.Router.prototype.execute = function(callback, args) {
    // Make sure only monitor is loaded before connection is made
    if (checkConnection())
        return

    if (Backbone.history.getHash().match(/admin*/))
        app.LayoutInstance.showAdminNav()
    else
        app.LayoutInstance.showNav()

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