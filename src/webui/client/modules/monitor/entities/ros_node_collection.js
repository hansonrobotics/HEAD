define(['application', 'lib/api'], function (App, api) {
    return Backbone.Collection.extend({
        comparator: 'name',
        config: {},
        fetch: function () {
            var self = this;
            $.ajax({
                url: "/monitor/status",
                dataType: "json",
                success: function (data) {
                    self.config = data;
                    // Reset collection after setting config to redraw view
                    self.reset();
                    _.each(data.nodes, function (attrs) {
                        self.add(new Entities.RosNode(attrs));
                    });
                },
                error: function () {
                    self.config = {};
                    self.reset();
                }
            });
        }
    });
});
