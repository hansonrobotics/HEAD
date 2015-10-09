define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Log = Backbone.Model.extend({
            initialize: function () {
            },
        });
        Entities.LogCollection = Backbone.Collection.extend({
            model: Entities.Log,
            fetch: function () {
                var self = this;
                $.ajax('/monitor/logs/warn', {
                    dataType: 'json',
                    success: function (response) {
                        self.add(response);
                        console.log(response)
                    }
                });
            }
        });
    });
});

