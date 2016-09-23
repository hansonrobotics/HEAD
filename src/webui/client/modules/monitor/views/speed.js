define(['application', './templates/speed.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Speed = Marionette.ItemView.extend({
            template: template
        });
    });

    return App.module('Monitor.Views').Speed;
});
