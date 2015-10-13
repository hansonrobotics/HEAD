define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Log = Backbone.Model.extend({
            idAttribute: 'node'
        });
        Entities.LogCollection = Backbone.Collection.extend({
            initialize: function () {
                self.lastUpdated = new Date(new Date().setDate(new Date().getDate()-1));
            },
            model: Entities.Log,
            fetch: function () {
                var self = this;
                $.ajax('/monitor/logs/warn/'+self.lastUpdated, {
                    dataType: 'json',
                    success: function (response) {
                        console.log(self);
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
                        self.add(response.logs);
                        self.lastUpdated = response.timestamp
                    }
                });
            }
        });
    });
});

