define(['application', 'backbone', 'lib/api', 'path'], function (App, Backbone, api, path) {
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
            return typeof this.performancePath === 'string' ? path.join('/performance/attention', this.performancePath)
                : '/attention_regions/' + api.config.robot
        },
        save: function (options) {
            this.sync("create", this, options);
        },
        sync: function (method, model, options) {
            options = options || {};
            if (_.includes(['create', 'update'], method)) {
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
        },
        setPerformancePath: function (path) {
            this.performancePath = path;
        }
    });
});
