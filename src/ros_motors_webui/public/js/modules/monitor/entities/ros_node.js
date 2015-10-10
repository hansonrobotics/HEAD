define(['application', 'lib/api'], function (App, api) {
    App.module('Monitor.Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.RosNode = Backbone.Model.extend({
            stop: function(){},
            start: function(){},
        });

        Entities.RosNodeCollection = Backbone.Collection.extend({
            model: Entities.RosNode,
            comparator: 'name',
            config: {},
            fetch: function () {
                var self = this;
                $.ajax({
                    url: "/monitor/status",
                    dataType: "json",
                    success: function (data) {
                        self.config = _.extend(data);
                        // Reset collection after setting config to redraw view
                        self.reset();
                        _.each(data.nodes, function(attrs){
                            self.add(new Entities.RosNode(attrs));
                        });
                    }
                 });
            }
        });
    });
});
