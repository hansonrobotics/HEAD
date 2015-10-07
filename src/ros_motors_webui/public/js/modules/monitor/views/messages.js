define(['application', 'tpl!./templates/messages.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Messages = Marionette.ItemView.extend({
            template: template
        });
    });

    return App.module('Monitor.Views').Messages;
});
