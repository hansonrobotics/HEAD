define(['application', 'lib/api'], function (App, api) {
    App.module('Start.Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Check = Backbone.Model.extend({});

        Entities.CheckCollection = Backbone.Collection.extend({
            model: Entities.Check,
            comparator: 'name',
            config: {},
            fetch: function (action) {
                if (typeof action === 'undefined') { action = false; }
                var url = "/monitor/start_status"
                if (action){
                    url += "/" + action
                }
                var self = this;
                $.ajax({
                    url: url,
                    dataType: "json",
                    success: function (data) {
                        self.config = data;
                        // Reset collection after setting config to redraw view
                        self.reset();
                        _.each(data.checks, function(attrs){
                            self.add(new Entities.Check(attrs));
                        });
                    },
                    error: function(){
                        self.config = {};
                        self.reset();
                    }
                 });
            }
        });
    });
});
