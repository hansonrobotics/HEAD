define(['application', 'tpl!./templates/log.tpl'], function (App, template) {
    App.module('Monitor.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Logs = Marionette.ItemView.extend({
            template: template,
            ui: {
                label: '.pull-right',
                title: '.title',
                body: '.panel-body',
                collapse: '.panel-collapse',
                a: '.collapsed',
            },
            onRender: function () {
                var self = this;
                var node = self.model.get('node')
                var log = self.model.get('log')
                self.ui.label.text(log.length);
                self.ui.a.attr("href", "#"+node);
                self.ui.collapse.attr("id", node);
                self.ui.body.text(log);
            },
        });
    });

    return App.module('Monitor.Views').Logs;
});

