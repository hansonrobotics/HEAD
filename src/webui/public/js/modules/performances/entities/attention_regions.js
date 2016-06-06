define(['application', 'backbone', 'lib/api', 'underscore', 'jquery'], function (App, Backbone, api, _, $) {
    var AttentionRegion = Backbone.Model.extend({
        toJSON: function (options) {
            var json = Backbone.Model.prototype.toJSON.call(this, options);
            delete json['id'];
            return json;
        }
    });
    return Backbone.Collection.extend({
        comparator: 'id',
        model: AttentionRegion,
        url: function () {
            return '/attention_regions/' + api.config.robot
        },
        save: function (options) {
            this.sync("create", this, options);
        },
        sync: function (method, model, options) {
            options = options || {};
            if (_.contains(['create', 'update'], method)) {
                $.ajax(this.url(), {
                    method: 'POST',
                    data: JSON.stringify(this.toJSON()),
                    success: function (response) {
                        options.success && options.success(response);
                    },
                    error: function (error) {
                        options.error && options.error(error);
                    }
                });
            } else {
                Backbone.sync(method, model, options);
            }
        }
    });
});
