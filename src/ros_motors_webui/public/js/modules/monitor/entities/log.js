define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Log = Backbone.Model.extend({
            idAttribute: 'node'
        });
        Entities.LogCollection = Backbone.Collection.extend({
            initialize: function () {
                this.cursors = {};
            },
            model: Entities.Log,
            fetch: function () {
                var self = this;
                $.ajax('/monitor/logs/warn', {
                    method: 'POST',
                    dataType: 'json',
                    data: self.cursors,
                    success: function (response) {
                        _.each(response.logs, function(log){
                            var m = self.get(log.node)
                            if (m){
                                m.new_logs = log.log;
                                m.trigger("log:changed");
                                console.log("log changed");
                            }else{
                                console.log("log added");
                                self.add(log);
                            }
                        });
                        self.cursors = response.cursors;
                    }
                });
            }
        });
    });
});

