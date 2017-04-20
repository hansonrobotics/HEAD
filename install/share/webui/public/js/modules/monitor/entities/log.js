define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Log = Backbone.Model.extend({
            idAttribute: 'node',
            initialize: function (options) {
                console.log(options);
                console.log(this);
            }
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
                        _.each(response.logs, function (log) {
                            var m = self.get(log.node);
                            if (m) {
                                m.new_logs = log.log;
                                m.trigger("log:changed");
                                console.log("log changed");
                            } else {
                                console.log("log added");
                                self.add(log);
                            }
                        });
                        self.cursors = response.cursors;
                        self.add(self.getCompositeModel(), {at: 0});
                    }
                });
            },
            getCompositeModel: function () {
                var logs = [];
                this.each(function (model) {
                    Array.prototype.push.apply(logs, model.get('log'));
                });

                logs = _.sortBy(logs, function (log) {
                    return log['asctime'];
                }).reverse();

                var model = new Entities.Log({
                    node: 'global_log',
                    log: logs
                });

                return model;
            }
        });
    });
});

