define(['application', 'tpl!./templates/system.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.System = Marionette.ItemView.extend({
            template: template
        });
    });

    return App.module('Monitor.Views').System;
});
