define(['backbone', 'lib/api'], function (Backbone, api) {
    return Backbone.Model.extend({
        url: '/robot_config/' + api.config.robot
    });
});
