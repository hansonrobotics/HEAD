define(['application', 'lib/api', './log'], function (App, api, Log) {
    return Backbone.Collection.extend({
        initialize: function () {
            this.cursors = {};
        },
        model: Log,
        fetch: function () {
            var self = this;
            $.ajax('/monitor/logs/warn', {
                method: 'POST',
                dataType: 'json',
                data: JSON.stringify(self.cursors),
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

            var model = new Log({
                node: 'global_log',
                log: logs
            });

            return model;
        }
    });
});

