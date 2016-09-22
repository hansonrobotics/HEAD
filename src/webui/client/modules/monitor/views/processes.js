define(['application', './templates/processes.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Processes = Marionette.ItemView.extend({
            template: template
        });
    });

    return App.module('Monitor.Views').Processes;
});
