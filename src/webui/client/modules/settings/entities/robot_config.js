define(['backbone', 'lib/api'], function (Backbone, api) {
    return Backbone.Model.extend({
        url: function () {
            return '/robot_config/' + api.config.robot;
        }
    });
});
